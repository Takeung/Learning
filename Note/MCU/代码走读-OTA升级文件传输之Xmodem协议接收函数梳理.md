## 代码走读-OTA升级文件传输之Xmodem协议接收函数梳理

## 接收方

```c
/*****************************************************************************
* Local pre-processor symbols/macros ('define')
*****************************************************************************/
#define _inbyte xmodem_read_byte
#define _outbyte xmodem_write_byte

#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18

#define DLY_20S 20000
#define MAXRETRANS 25

#define USER_LED_PORT           CY_LED0_PORT
#define USER_LED_PIN            CY_LED0_PIN
#define USER_LED_PIN_MUX        CY_LED0_PIN_MUX

/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/
static uint32_t totalSize = 0;
static uint8_t xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */

static uint32_t flushcharcount = 0;
static uint8_t flushbuff[1030];

/*****************************************************************************
* Local function prototypes ('static')                                                                            
*****************************************************************************/
static int xmodem_read_byte(int16_t timeout_ms);
static void xmodem_write_byte(uint8_t c);
static void xmodem_flushinput(void);
static unsigned short xmodem_crc16_ccitt(const uint8_t *buf, int sz);
static uint8_t xmodem_check(uint8_t crc_mode, const uint8_t *buf, int sz);
static uint8_t xmodem_flashWrite(uint8_t *data, uint32_t start_addr, uint32_t len);

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/
static int xmodem_read_byte(int16_t timeout_ms)
{
    uint8_t u8ReadBuf;
    uint16_t u16ReadCnt;
    
    // Receive data from UART asynchrnously (Non-blocking)
    for (;;) 
    {
        if (1UL == Cy_SCB_UART_GetArray(User_USB_SCB8_TYPE, &u8ReadBuf, 1)) 
        {
            u16ReadCnt = 1;
            break;
        }
        
        if (timeout_ms-- <= 0)
        {
            u16ReadCnt = 0;
            break;
        }
    }
    
    return ((u16ReadCnt == 0) ? -1 : u8ReadBuf);
}

static void xmodem_write_byte(uint8_t c)
{    
    Cy_SCB_UART_PutArray(User_USB_SCB8_TYPE, &c, 1);  
}

static void xmodem_flushinput(void)
{
    int c;
    // flush and wait for 3 second without data received
    flushcharcount = 0;
    
    while ((c=_inbyte(DLY_20S)) >= 0)
    {
        if (flushcharcount < 1030) 
        {
            flushbuff[flushcharcount] = c;
        }
        flushcharcount++;
    }
}

static uint16_t xmodem_crc16_ccitt(const uint8_t *buf, int sz)
{
    unsigned short crc = 0;
    while (--sz >= 0) {
        int i;
        crc ^= (uint16_t) *buf++ << 8;
        for (i = 0; i < 8; i++)
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc <<= 1;        
    }
    return crc;
}

static uint8_t xmodem_check(uint8_t crc_mode, const uint8_t *buf, int sz)
{
    if (crc_mode) { // CRC mode
        uint16_t crc = xmodem_crc16_ccitt(buf, sz);
        uint16_t tcrc = (buf[sz]<<8)+buf[sz+1];
        if (crc == tcrc)
        {
            return true;
        }
    }
    else { // checksum mode
        int i;
        uint8_t cks = 0;
        for (i = 0; i < sz; ++i) {
            cks += buf[i];
        }
        if (cks == buf[sz])
        {
            return true;
        }
    }
    return false;
}

static uint8_t xmodem_flashWrite(uint8_t *data, uint32_t start_addr, uint32_t len)
{
    uint8_t ret;
    unsigned short crc1;
    unsigned short crc2;
    uint32_t* pProgramData;

    //Programming code flash
    Cy_FlashInit(false /*blocking*/);   

    pProgramData = (uint32_t*)data; 

    for(uint32_t i_addr = start_addr; i_addr < start_addr + len; i_addr+=512) //program 512byte       
    {
     FOTA_ProgramFlash(i_addr, 512, pProgramData);//program 512 byte          
     pProgramData=pProgramData+128;//4x128
    }
    //Verify
    uint32_t* pVerifySrc= (uint32_t*)data;
    uint32_t* pVerifyDest = (uint32_t*)start_addr;

    for(uint32_t i_wordId = start_addr; i_wordId < len; i_wordId++)
    {
        CY_ASSERT(pVerifyDest[i_wordId] == pVerifySrc[i_wordId]); 
    }

    crc1 = xmodem_crc16_ccitt(data, len);
    //crc2 = crc16_ccitt((uint8_t const*)start_addr, len);    
    crc2 = xmodem_crc16_ccitt(data, len);
    if (crc1 == crc2)
    {
        ret = true;
    }
    else
    {
        ret = false;
    }
    
    return ret;
}

int32_t Xmodem_receive(uint8_t *dest, uint32_t receive_buffsize, uint32_t start_addr, uint32_t max_total)
{
    uint32_t flash_address;
    unsigned char *p;
    int bufsz, crc_mode = 0;
    unsigned char trychar = 'C';
    unsigned char packetno = 1;
    int i, len = 0;
    int c = 0;
    int retry, retrans = MAXRETRANS;
    uint8_t ret;

    totalSize = 0;

    flash_address = start_addr;  // flash starting address

    // Term_Printf("Xmodem: Starting data reception.\r\n");

    for(;;) 
    {
        // Clear hardware watchdog
        // ClearWatchdog();
      
        for (retry = 0; retry < 5; ++retry)  // Retry up to 5 times for synchronization
        { 
            if (trychar)
            {
                // Term_Printf("Xmodem: Sending start character: 0x%X\r\n", trychar);
                _outbyte(trychar);    // Write data to UART        
            }

            if ((c = _inbyte(DLY_20S)) >= 0) 
            {
                switch (c) 
                {
                    case SOH:  // 0x01
                        bufsz = 128;
                        // Term_Printf("Xmodem: SOH received, packet size 128 bytes.\r\n");
                        goto start_recv;
                    case STX:  // 0x02
                        bufsz = 1024;
                        // Term_Printf("Xmodem: STX received, packet size 1024 bytes.\r\n");
                        goto start_recv;
                    case EOT:  // 0x04
                        // Term_Printf("Xmodem: EOT received, ending transmission.\r\n");
                        xmodem_flushinput();
                        _outbyte(ACK);
                        Cy_GPIO_Set(USER_LED_PORT, USER_LED_PIN);    

                        if (max_total >= (totalSize + len)) 
                        {
                            ret = xmodem_flashWrite(dest, flash_address, len);
                        } 
                        else 
                        {
                            ret = false;
                        }                            

                        Cy_GPIO_Clr(USER_LED_PORT, USER_LED_PIN);       

                        if (ret == false)
                        {
                            // Term_Printf("Xmodem: Flash write error.\r\n");
                            xmodem_flushinput();
                            _outbyte(CAN);
                            _outbyte(CAN);
                            _outbyte(CAN);                        
                            return XMODEM_RESULT_FLASH_ERROR;  // Flash error
                        }

                        flash_address += len;
                        totalSize += len;
                        Term_Printf("Xmodem: Total size received: %d bytes.\r\n", totalSize);
                        return totalSize;  // Normal end
                    case CAN:  // 0x18
                        if ((c = _inbyte(DLY_20S)) == CAN) 
                        {
                            Term_Printf("Xmodem: Cancel signal received.\r\n");
                            xmodem_flushinput();
                            _outbyte(ACK);
                            return XMODEM_RESULT_REMOTE_CANCEL;  // Canceled by remote
                        }
                        break;
                    default:
                        if (trychar == 0) 
                        {
                            Term_Printf("Xmodem: Unknown char received during sync: 0x%X\r\n", c);
                            xmodem_flushinput();
                            _outbyte(NAK);  // Request sender to retransmit
                        }
                        break;
                }
            }

            if (trychar == 0)  
            { 
                _outbyte(NAK);  // Request sender to retransmit package
            }
        }

        if (trychar == 'C')
        { 
            trychar = NAK; 
            continue; 
        }

        Term_Printf("Xmodem: Sync error, sending CAN.\r\n");
        xmodem_flushinput();
        _outbyte(CAN);
        _outbyte(CAN);
        _outbyte(CAN);
        return XMODEM_RESULT_SYNC_ERROR;  // Sync error

    start_recv: 
        if (trychar == 'C') crc_mode = 1;
        trychar = 0;
        p = xbuff;
        *p++ = c;

        for (i = 0; i < (bufsz + (crc_mode ? 1 : 0) + 3); ++i) 
        {
            if ((c = _inbyte(DLY_20S)) >= 0) 
            {
                *p++ = c;
            } 
            else 
            {
                break;
            }
        }

        if (i != (bufsz + (crc_mode ? 1 : 0) + 3))  // Missing data for a complete package
        {
            Term_Printf("Xmodem: Incomplete packet received, sending NAK.\r\n");
            _outbyte(NAK);  // Request retransmission
        } 
        else if (!((xbuff[1] == (unsigned char)(~xbuff[2])) && xmodem_check(crc_mode, &xbuff[3], bufsz)))  // Invalid package data
        {
            Term_Printf("Xmodem: Invalid packet data received, sending NAK.\r\n");
            xmodem_flushinput();
            _outbyte(NAK);  // Request retransmission
        } 
        else if (!((xbuff[1] == packetno) || (xbuff[1] == (unsigned char)packetno - 1))) 
        {
            Term_Printf("Xmodem: Incorrect packet number received, sending NAK.\r\n");
            xmodem_flushinput();
            _outbyte(NAK);  // Request retransmission
        } 
        else 
        {
            if (xbuff[1] == packetno) 
            {
                register int count = receive_buffsize - len;                
                if (count > bufsz) 
                    count = bufsz;
                if (count > 0) 
                {
                    memcpy(&dest[len], &xbuff[3], count);
                    len += count;
                }                
                if (receive_buffsize == len)
                {
                    // Term_Printf("Xmodem: Buffer full, writing to flash.\r\n");
                    Cy_GPIO_Set(USER_LED_PORT, USER_LED_PIN);      

                    if (max_total >= (totalSize + len))                    
                        ret = xmodem_flashWrite(dest, flash_address, len);
                    else
                        ret = false;

                    Cy_GPIO_Clr(USER_LED_PORT, USER_LED_PIN);

                    if (ret == false)
                    {
                        Term_Printf("Xmodem: Flash write error during buffer full state.\r\n");
                        xmodem_flushinput();
                        _outbyte(CAN);
                        _outbyte(CAN);
                        _outbyte(CAN);
                        return XMODEM_RESULT_FLASH_ERROR;  // Flash error
                    }

                    flash_address += len;
                    totalSize += len;
                    len = 0;
                }
                ++packetno;
                retrans = MAXRETRANS + 1;  // Reset retransmission counter
            }

            if (--retrans <= 0) 
            {
                Term_Printf("Xmodem: Too many retransmissions, sending CAN.\r\n");
                xmodem_flushinput();
                _outbyte(CAN);
                _outbyte(CAN);
                _outbyte(CAN);
                return XMODEM_RESULT_TOO_MANY_RETRAN;  // Too many retries
            }

            _outbyte(ACK);  // Send acknowledgment
            // Term_Printf("Xmodem: The %uth Package received success.\r\n", packetno);
        }
    }
}
```

该函数 `Xmodem_receive` 的作用是实现使用 **XMODEM协议** 来通过串口接收数据，并将接收到的数据写入到指定的存储区域（例如闪存）。XMODEM 是一种简单的文件传输协议，用于在两台设备之间可靠地传输数据。这个函数执行的具体任务是接收数据包，校验数据的完整性，然后将数据写入到目标内存区域。下面是该函数的详细分解：

### 函数功能
`Xmodem_receive` 通过 XMODEM 协议从串口接收数据，并将数据存储在指定的缓冲区 (`dest`) 中。如果数据包大小超出缓冲区限制，数据会写入到闪存 (`flash_address`) 中。

### 函数参数
- `uint8_t *dest`: 指向接收数据的目标缓冲区。
- `uint32_t receive_buffsize`: 缓冲区的大小，防止缓冲区溢出。
- `uint32_t start_addr`: 接收的数据将写入的闪存起始地址。
- `uint32_t max_total`: 数据传输的最大大小，防止写入超出总容量。

### 函数流程概述
1. **初始化和配置**
   - `flash_address` 是闪存的起始地址。函数开始时，将 `start_addr` 赋值给 `flash_address`。
   - 使用一个字符 `trychar` 启动通信，默认值为 `'C'` 表示尝试启用 **CRC 校验模式**。

2. **通信同步和初步握手**
   - 函数尝试通过发送 `'C'`（CRC 模式）或 `NAK`（简单校验模式）来与发送方进行握手，等待接收方的响应。
   - 超时时间根据是否在初次同步时会有所不同，首次同步超时时间较短（3秒），后续包同步则较长（10秒）。

3. **处理接收到的数据包**
   - 当接收到 `SOH` (128字节数据包) 或 `STX` (1024字节数据包) 时，进入接收数据包阶段。
   - 如果接收到 `EOT` (传输结束标志)，则结束接收并返回正常结束。
   - 如果接收到 `CAN` (取消传输)，则处理取消操作。

4. **数据校验与确认**
   - 在接收到数据包后，函数会根据当前是 CRC 模式还是校验和模式来检查数据包的完整性。
   - 如果数据包校验通过，并且包序号正确，则数据会被存入目标缓冲区。
   - 当缓冲区满时，会将数据写入闪存。

5. **错误处理与重传**
   - 如果接收到的数据包有错误或缺少数据，函数会请求重传，最多允许重传 `MAXRETRANS` 次。
   - 如果重试次数超过上限或其他严重错误，函数将终止传输，并返回错误代码。

6. **数据写入闪存**
   - 每当缓冲区满或收到结束标志 `EOT` 时，函数会将数据写入闪存。如果写入失败，将中断传输并返回闪存写入错误。

### 重要状态和变量
- `bufsz`: 存储每个数据包的大小（128 字节或 1024 字节）。
- `packetno`: 当前的数据包序号。XMODEM 协议通过序号来确保包的顺序。
- `totalSize`: 已接收的总字节数。
- `crc_mode`: 用于标记当前是 CRC 校验模式还是校验和模式。
- `retrans`: 当前允许的重传次数，在每次接收成功时重置为最大重传次数 `MAXRETRANS`。
- `error_packetsize`, `error_packetdata`, `error_packetno`, `error_packetnoresend`: 用于记录不同类型的错误统计。
  
### 特殊的返回值
- `XMODEM_RESULT_FLASH_ERROR`: 闪存写入错误。
- `XMODEM_RESULT_REMOTE_CANCEL`: 远程发送方取消传输。
- `XMODEM_RESULT_SYNC_ERROR`: 同步错误。
- `XMODEM_RESULT_TOO_MANY_RETRAN`: 重传次数过多错误。

### 总结
`Xmodem_receive` 实现了一个完整的 XMODEM 接收流程，功能包括：
- 与发送方的握手和同步。
- 数据包的接收、校验和重传请求。
- 处理传输结束或取消指令。
- 错误处理和统计。
- 将数据写入到缓冲区或闪存中。



## 发送方

```c
int send_file(int fd, const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("fopen");
        return -1;
    }

    uint8_t block_number = 1;
    uint8_t buffer[BLOCK_SIZE];
    uint8_t packet[BLOCK_SIZE + 5];

    // Wait for NAK from receiver
    char ch;
    do {
        if (read(fd, &ch, 1) < 0) {
            perror("read");
            fclose(file);
            return -1;
        }
    } while (keep_running && (ch != CRCCHR));

    while (keep_running) {
        size_t bytes_read = fread(buffer, 1, BLOCK_SIZE, file);
        if (bytes_read == 0) {
            break; // No more data to send
        }

        packet[0] = SOH;
        packet[1] = block_number;
        packet[2] = ~block_number;
        memcpy(&packet[3], buffer, bytes_read);

        if (bytes_read < BLOCK_SIZE) {
            memset(&packet[3 + bytes_read], 0x1A, BLOCK_SIZE - bytes_read); // Padding with 0x1A
        }

        uint16_t crc = crc16_ccitt(&packet[3], BLOCK_SIZE);
        packet[BLOCK_SIZE + 3] = crc >> 8;
        packet[BLOCK_SIZE + 4] = crc & 0xFF;

        if (write(fd, packet, BLOCK_SIZE + 5) < 0) {
            perror("write");
            fclose(file);
            return -1;
        }

        if (read(fd, &ch, 1) < 0) {
            perror("read");
            fclose(file);
            return -1;
        }

        if (ch == ACK) {
            block_number++;
        } else if (ch == NAK) {
            fseek(file, -BLOCK_SIZE, SEEK_CUR); // Resend the current block
        } else {
            fprintf(stderr, "Unexpected response: %c\n", ch);
            fclose(file);
            return -1;
        }
    }
    char eot = EOT;
    // Send EOT
    while (keep_running) {
        if (write(fd, &eot, 1) < 0) {
            perror("write");
            fclose(file);
            return -1;
        }

        if (read(fd, &ch, 1) < 0) {
            perror("read");
            fclose(file);
            return -1;
        }

        if (ch == ACK) {
            break;
        }
    }

    fclose(file);
    return 0;
}
```

### `send_file` 函数的作用

该函数的目的是通过 **XMODEM 协议** 将文件从发送方（通常是本地设备）通过串口发送到接收方设备。文件被分成固定大小的块，每块加上校验码后发送到接收设备。发送的每一块数据，接收方都会进行校验并通过 **ACK**（确认）或者 **NAK**（重传请求）进行回应。最后，当文件发送完毕，发送方通过 **EOT**（传输结束标志）通知接收方传输结束。

### 函数参数

- `int fd`: 发送数据的文件描述符，通常是串口文件描述符。
- `const char *filename`: 需要发送的文件路径名。

### 函数流程概述

1. **打开文件**：
   - 使用 `fopen` 函数以二进制读模式打开文件。如果打开失败，函数返回错误。

2. **等待接收方的响应**：
   - 发送方等待接收方通过发送 **NAK** 或者 **CRCCHR**（表示使用 CRC 校验模式）来表示已经准备好接收数据。在这个阶段，发送方通过串口读取数据，直到接收到 **CRCCHR** 符号，表示接收方希望使用 CRC 校验来传输数据。

3. **逐个块发送文件**：
   - 文件被分成固定大小的块（`BLOCK_SIZE`，通常是128字节或1024字节），每个块被打包为数据包。
   - 数据包的格式为：
     1. **SOH**: 标志数据包的起始，通常为 0x01。
     2. **包序号**: 发送方的块序号，避免数据重复。
     3. **包序号补码**: 包序号的反码，用于简单的错误检查。
     4. **数据块**: 从文件中读取的实际数据，大小为 `BLOCK_SIZE`。
     5. **CRC 校验码**: 发送方对数据块进行 CRC 校验，保证数据传输的正确性。
   - 如果读取的文件大小小于块大小，则剩余部分用 `0x1A` 填充。

4. **数据发送和响应处理**：
   - 每发送一个数据包后，发送方等待接收方的响应。接收方会返回：
     - **ACK**: 表示接收成功，发送方继续发送下一个数据包。
     - **NAK**: 表示接收错误，发送方重传当前数据包。
     - 其他字符表示意外情况，函数将终止发送并返回错误。

5. **结束传输**：
   - 文件发送完成后，发送方会发送一个 **EOT**（传输结束标志，通常为 0x04）来通知接收方传输结束。
   - 发送方持续发送 **EOT**，直到接收到接收方的 **ACK**，确认接收方同意结束。

6. **关闭文件**：
   - 文件传输结束后，关闭文件并返回 0 表示传输成功。

### 与 `Xmodem_receive` 函数的对应关系

`send_file` 和 `Xmodem_receive` 是 **XMODEM 协议** 的两个对等部分，分别在发送方和接收方实现了协议的功能。它们的相互作用过程如下：

1. **初始同步**：
   - `Xmodem_receive` 函数等待来自发送方的 `CRCCHR` 或 `NAK`，用来决定是使用 CRC 还是校验和模式。
   - `send_file` 函数在发送数据前，会等待接收方发送 **CRCCHR**，表明接收方已经准备好接收。

2. **数据包传输**：
   - 在发送数据包时，`send_file` 函数发送数据块并计算其 CRC 校验值。数据包包括序号、序号的补码、数据块以及 CRC 校验。
   - `Xmodem_receive` 函数在接收到数据包后，会对数据进行校验（根据接收方是否使用 CRC）。如果校验正确，接收方发送 **ACK** 确认；如果校验失败，发送 **NAK** 请求重传。

3. **数据包重传**：
   - 当 `send_file` 函数收到 **NAK** 时，会重新发送当前数据包。
   - `Xmodem_receive` 函数检测到数据包错误时，发送 **NAK** 让发送方重传。

4. **传输结束**：
   - `send_file` 函数在发送完所有数据块后，发送 **EOT**，表示传输结束。
   - `Xmodem_receive` 在接收到 **EOT** 后，确认并写入接收到的数据到闪存，并发送 **ACK** 结束。

### 函数的关键点

- **校验方式**：XMODEM 支持两种校验方式，CRC 校验和校验和模式。`send_file` 和 `Xmodem_receive` 通过在开始时发送和接收 **CRCCHR** 来确认使用 CRC 校验。
- **包的完整性检查**：每个数据包发送后，接收方会通过 **ACK** 确认包正确到达，或者通过 **NAK** 请求重传。`send_file` 函数根据接收方的响应来判断是否继续发送下一个包或者重传当前包。
- **错误处理与重传**：`Xmodem_receive` 如果接收到的包序号或数据校验不正确，会请求重传。`send_file` 会根据接收方的要求重传数据包。
- **传输结束**：通过 **EOT** 来表示文件传输结束，接收方确认后，发送方结束传输。

### 总结

`send_file` 和 `Xmodem_receive` 是基于 **XMODEM 协议** 的文件传输函数，分别处理发送和接收。通过校验包序号和数据完整性，它们确保数据在串口上的可靠传输。

### 传输优化

鉴于接收端和发送端OTA过程中常见同步错误的问题，主要在于建链过程中请求与响应缺乏同步机制，故而需要延长响应的等待时间，修改后实测稳定性极大增强。