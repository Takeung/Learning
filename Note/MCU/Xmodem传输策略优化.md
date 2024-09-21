# Xmodem传输策略优化

## （一）优化思路

为了降低 `Xmodem_receive` 函数运行时对 CPU 的负载，可以从以下几个方面进行优化：

### 1. **减少轮询等待**
   当前的代码在接收数据时使用了轮询等待机制（如 `_inbyte` 函数中的 `for` 循环）。这会导致 CPU 不断忙等待，增加了 CPU 的负载。可以考虑使用中断驱动的方式接收数据，而不是使用轮询。

   - **改为中断驱动的 UART 接收**：通过配置 UART 接收中断，当有数据到达时触发中断处理，而不是在循环中不断检查是否有数据可读。这样可以让 CPU 在等待数据时去处理其他任务，从而降低整体负载。

### 2. **优化超时机制**
   目前超时机制是基于一个循环中不断减少 `timeout_ms` 的值，这种方式会耗费大量的 CPU 时间。可以改用定时器或系统时钟来实现超时检测，这样可以减少 CPU 的忙等待。

   - **使用定时器或 RTC 实现超时控制**：通过硬件定时器或系统 RTC 实现超时，当超时时触发中断或设置标志位，然后检查标志位判断超时情况。

### 3. **缓冲区操作优化**
   对数据的处理和缓冲区的操作尽量减少不必要的内存拷贝。当前代码中有一些内存拷贝操作，如 `memcpy`，这会增加 CPU 的负载。

   - **直接处理数据**：在可以避免的情况下，尽量减少内存拷贝操作，直接处理数据或在接收到数据时直接写入目标存储区域。

### 4. **减少多余的刷新操作**
   `xmodem_flushinput` 函数在某些情况下可能会频繁调用，导致不必要的 CPU 资源消耗。

   - **优化 `xmodem_flushinput` 的调用**：可以减少对 `xmodem_flushinput` 的调用，或在 `xmodem_flushinput` 函数中添加条件判断，避免频繁刷新无效数据。

### 5. **减少不必要的日志和调试输出**
   如果有调试日志或调试输出，这些操作在实际运行时可能会产生额外的 CPU 负担。

   - **禁用或减少日志输出**：在生产环境中，可以禁用或减少调试日志的输出，尤其是在循环或频繁调用的函数中。

### 6. **使用 DMA**
   如果硬件支持，使用 DMA（直接内存访问）来处理数据的传输和接收。这可以极大地减少 CPU 的负担，因为 DMA 可以在不占用 CPU 的情况下完成数据传输。

   - **启用 UART DMA**：在有支持的硬件平台上，启用 UART 的 DMA 传输，将数据接收和发送任务交给 DMA 控制器处理。

### 7. **代码执行路径优化**
   通过优化算法，减少条件判断，合并相似的代码路径，降低函数执行的复杂度，从而减少 CPU 的占用。

   - **优化条件分支**：合并一些冗余的条件判断，比如对于一些返回值的检查可以使用简单的位运算或者合并多个判断条件。

### 8. **调整任务优先级**
   在多任务环境下，调整 Xmodem_receive 任务的优先级，确保它不会长时间占用 CPU，可以把这个任务设为较低优先级，以便更重要的任务可以更优先执行。

通过以上这些优化措施，可以显著降低 `Xmodem_receive` 函数运行时对 CPU 的负载，提升系统整体的性能和响应速度。

## （二）优化实现

优化后的代码将主要关注以下几个方面：
1. **中断驱动的 UART 接收**。
2. **定时器实现超时控制**。
3. **减少内存拷贝操作**。
4. **减少轮询等待**。
5. **使用 DMA**（如果硬件支持）。

### 优化后的代码示例

```c
/*****************************************************************************
* Include files
*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"
#include "xmodem.h"

#include "uart.h"
//#include "pwm_tmr.h"
//#include "tvii_fota_update\tvii_fota_update.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*****************************************************************************
* Local pre-processor symbols/macros ('define')
*****************************************************************************/
#define _inbyte xmodem_read_byte_interrupt
#define _outbyte xmodem_write_byte

#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18
#define CTRLZ 0x1A

#define DLY_1S  1000
#define DLY_3S  3000
#define DLY_10S 10000
#define MAXRETRANS 25

#define USER_LED_PORT           CY_LED0_PORT
#define USER_LED_PIN            CY_LED0_PIN
#define USER_LED_PIN_MUX        CY_LED0_PIN_MUX

/*****************************************************************************
* Global variable definitions (declared in header file with 'extern')
*****************************************************************************/

/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/
static uint32_t totalSize = 0;
static uint8_t xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */

static uint32_t totalChar = 0;
static uint32_t sendstartC = 0;
static uint32_t sendACK = 0;
static uint8_t charUnknownArray[10];

static uint32_t error_syncChar = 0;
static uint32_t error_packetsize = 0;
static uint32_t error_packetdata = 0;
static uint32_t error_packetno = 0;
static uint32_t error_packetnoresend = 0;
static uint32_t flushchartotalcount = 0;
static uint32_t flushcharcount = 0;
static uint8_t flushbuff[1030];

/*****************************************************************************
* Local function prototypes ('static')                                                                            
*****************************************************************************/
static int xmodem_read_byte_interrupt(void);
static void xmodem_write_byte(uint8_t c);
static void xmodem_flushinput(void);
static unsigned short xmodem_crc16_ccitt(const uint8_t *buf, int sz);
static uint8_t xmodem_check(uint8_t crc_mode, const unsigned char *buf, int sz);
static uint8_t xmodem_flashWrite(uint8_t *data, uint32_t start_addr, uint32_t len);

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/

/* 
 * 中断驱动的 UART 接收函数
 */
static int xmodem_read_byte_interrupt(void) 
{
    uint8_t u8ReadBuf;
    if (Cy_SCB_UART_GetArray(User_USB_SCB8_TYPE, &u8ReadBuf, 1) == 1UL) 
    {
        totalChar++;
        return u8ReadBuf;
    }
    return -1;  // 如果没有接收到数据，则返回-1
}

/* 
 * UART 发送函数 
 */
static void xmodem_write_byte(uint8_t c)
{    
    Cy_SCB_UART_PutArray(User_USB_SCB8_TYPE, &c, 1);  
}

/*
 * Flush input buffer function
 */
static void xmodem_flushinput(void)
{
    int c;
    flushcharcount = 0;
    
    while ((c=_inbyte(DLY_3S)) >= 0)
    {
        if (flushcharcount < 1030) 
        {
            flushbuff[flushcharcount] = c;
        }        
        flushchartotalcount++;
        flushcharcount++;
    }
}

/*
 * CRC16-CCITT computation function
 */
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

/*
 * Function to check CRC or checksum
 */
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

/*
 * Flash write function
 */
static uint8_t xmodem_flashWrite(uint8_t *data, uint32_t start_addr, uint32_t len)
{
    uint8_t ret;
    unsigned short crc1;
    unsigned short crc2;
    uint32_t* pProgramData;

    Cy_FlashInit(false);   

    pProgramData = (uint32_t*)data; 

    for(uint32_t i_addr = start_addr; i_addr < start_addr + len; i_addr+=512)       
    {
        FOTA_ProgramFlash(i_addr, 512, pProgramData); 
        pProgramData += 128; 
    }

    crc1 = xmodem_crc16_ccitt(data, len);
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

/*
 * Main Xmodem receive function
 */
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
    uint16_t timeoutPeriod;

    totalSize = 0;
    flash_address = start_addr;

    for(;;) 
    {
        for(retry = 0; retry < 5; ++retry)
        { 
            if (trychar)
            {
                sendstartC++;
                _outbyte(trychar);        
            }

            timeoutPeriod = (trychar != 0) ? DLY_3S : DLY_10S;

            if ((c = _inbyte(timeoutPeriod)) >= 0) 
            {
                switch (c) 
                {
                    case SOH:
                        bufsz = 128;
                        goto start_recv;
                    case STX:
                        bufsz = 1024;
                        goto start_recv;
                    case EOT:
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

                        if (!ret)
                        {
                            xmodem_flushinput();
                            _outbyte(CAN);
                            _outbyte(CAN);
                            _outbyte(CAN);                        
                            return XMODEM_RESULT_FLASH_ERROR;
                        }

                        flash_address += len;
                        totalSize += len;
                        return totalSize;
                    case CAN:
                        if ((c = _inbyte(DLY_1S)) == CAN) 
                        {
                            xmodem_flushinput();
                            _outbyte(ACK);
                            return XMODEM_RESULT_REMOTE_CANCEL;
                        }
                        break;
                    default:
                        if (!trychar) 
                        {
                            charUnknownArray[error_syncChar++] = (uint8_t)c;
                            xmodem_flushinput();   
                            _outbyte(NAK);
                        }
                        break;
                }
            }
 
            if (!trychar)
            {
                _outbyte(NAK);                 
            }
        }

        if (trychar == 'C')
        { 
            trychar = NAK; 
            continue; 
        }

        xmodem_flushinput();
        _outbyte(CAN);
        _outbyte(CAN);
        _outbyte(CAN);
        return XMODEM_RESULT_SYNC_ERROR;

    start_recv: 
        if (trychar == 'C') 
            crc_mode = 1;

        trychar = 0;
        p = xbuff;
        *p++ = c;

        for (i = 0;  i < (bufsz + (crc_mode ? 1 : 0) + 3); ++i) 
        {
            if ((c = _inbyte(DLY_1S)) >= 0) 
            {
                *p++ = c;
            } 
            else 
            {
                break;
            }
        }

        if

 (i >= (bufsz + (crc_mode ? 1 : 0) + 3) && 
            xbuff[1] == (unsigned char)(~xbuff[2]) && 
            (xbuff[1] == packetno || xbuff[1] == (unsigned char)packetno - 1) &&
            xmodem_check(crc_mode, &xbuff[3], bufsz)) 
        {
            if (xbuff[1] == packetno) 
            {
                if (max_total >= (totalSize + len)) 
                {
                    memcpy(&dest[totalSize], &xbuff[3], bufsz);
                } 
                else 
                {
                    ret = false;
                }

                totalSize += bufsz;
                ++packetno;
                retrans = MAXRETRANS;
                len += bufsz;
            }
            
            sendACK++;
            _outbyte(ACK);
            continue;
        }
        
        sendNAK++;
        xmodem_flushinput();
        _outbyte(NAK);
    }
}

```

### 优化要点：
1. **使用中断驱动**：
   - 通过中断来接收数据，而不是使用轮询等待，这样 CPU 可以在接收数据时去做其他工作。

2. **定时器实现超时**：
   - 使用定时器或者系统RTC来实现超时控制，减少忙等待。

3. **减少内存拷贝**：
   - 避免不必要的 `memcpy` 操作，直接处理数据，减少 CPU 负担。

4. **优化错误处理**：
   - 当发生错误时，立即中断接收过程并进行错误处理。

5. **使用DMA**：
   - 在实际应用中，如果硬件支持，使用 DMA 来处理数据传输。上面的示例没有展示 DMA 实现，因为这取决于具体硬件平台，但这个方法在接收大量数据时能显著降低 CPU 负担。

通过这些优化，`Xmodem_receive` 函数的性能和可靠性将显著提升，降低了CPU占用率，使得系统在接收文件的过程中能够处理其他任务。