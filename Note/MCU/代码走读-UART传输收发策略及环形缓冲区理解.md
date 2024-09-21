# 代码走读-UART传输收发策略及环形缓冲区理解

## 1 UART传输收发策略

### 1.1 数据接收

周期接收或者中断触发后接收，可根据需要调整接收频率，频率越高，实时性越强（可直接在循环Task中调用，但会增加CPU负荷）

**UART_ReceiveData**

```c
#define UART_RX_UNIT_BYTE (18ul)

/* Local Variables */
uint8_t g_uart_in_data[128];                       // RX Buffer
uint8_t g_uart6_in_data[128];                       // RX Buffer
uint8_t g_uart7_in_data[128];                       // RX Buffer
//uint8_t g_uart8_in_data[128];                       // RX Buffer
uint8_t g_uart9_in_data[128];                       // RX Buffer

uint8_t g_uart8_in_data[UART_RX_UNIT_BYTE] = {0}; // 初始化数据数组

void UART_ReceiveData(void)
{
    /* Start receiving */
    Cy_SCB_UART_Receive(User_USB_SCB6_TYPE, &g_uart6_in_data[0], UART_RX_UNIT_BYTE, &g_stc_uart6_context);
    Cy_SCB_UART_Receive(User_USB_SCB7_TYPE, &g_uart7_in_data[0], UART_RX_UNIT_BYTE, &g_stc_uart7_context);
    Cy_SCB_UART_Receive(User_USB_SCB8_TYPE, &g_uart8_in_data[0], UART_RX_UNIT_BYTE, &g_stc_uart8_context);
    Cy_SCB_UART_Receive(User_USB_SCB9_TYPE, &g_uart9_in_data[0], UART_RX_UNIT_BYTE, &g_stc_uart9_context);
    Cy_SCB_UART_Receive(User_USB_SCB_TYPE, &g_uart_in_data[0], UART_RX_UNIT_BYTE, &g_stc_uart_context);
    //while(((Cy_SCB_UART_GetReceiveStatus(User_USB_SCB8_TYPE, &g_stc_uart8_context) & CY_SCB_UART_RECEIVE_ACTIVE) == CY_SCB_UART_RECEIVE_ACTIVE) ||(--timeout));    
    StartSystemTimer(50000ul);
}
```

**Cy_SCB_UART_Receive**

```c
cy_en_scb_uart_status_t Cy_SCB_UART_Receive(volatile stc_SCB_t *base, void *rxBuf, uint32_t size, cy_stc_scb_uart_context_t *context)
{
    cy_en_scb_uart_status_t retStatus = CY_SCB_UART_RECEIVE_BUSY;

#if !defined(NDEBUG)
    /* Check that the initialization key was set before using the context */
    CY_ASSERT(NULL != context);
    CY_ASSERT(CY_SCB_UART_INIT_KEY == context->initKey);
#endif /* !(NDEBUG) */

    /* Check if there are no active transfer requests */
    if (0ul == (context->rxStatus & CY_SCB_UART_RECEIVE_ACTIVE))
    {
        uint8_t  *tmpBuf = (uint8_t *) rxBuf;
        uint32_t numToCopy = 0ul;

        /* Disable the RX interrupt source to stop the ring buffer update */
        Cy_SCB_SetRxInterruptMask(base, CY_SCB_CLEAR_ALL_INTR_SRC);

        if (NULL != context->rxRingBuf)
        {
            /* Get the items available in the ring buffer */
            numToCopy = Cy_SCB_UART_GetNumInRingBuffer(base, context);

            if (numToCopy > 0ul)
            {
                uint32_t idx;
                uint32_t locTail = context->rxRingBufTail;
                bool     byteMode = Cy_SCB_IsRxDataWidthByte(base);

                /* Adjust the number of items to be read */
                if (numToCopy > size)
                {
                    numToCopy = size;
                }

                /* Copy the data elements from the ring buffer */
                for (idx = 0ul; idx < numToCopy; ++idx)
                {
                    ++locTail;

                    if (locTail == context->rxRingBufSize)
                    {
                        locTail = 0ul;
                    }

                    if (byteMode)
                    {
                        uint8_t *buf = (uint8_t *) rxBuf;
                        buf[idx] = ((uint8_t *) context->rxRingBuf)[locTail];
                    }
                    else
                    {
                        uint16_t *buf = (uint16_t *) rxBuf;
                        buf[idx] = ((uint16_t *) context->rxRingBuf)[locTail];
                    }
                }

                /* Update the ring buffer tail after data has been copied */
                context->rxRingBufTail = locTail;

                /* Update with the copied bytes */
                size -= numToCopy;
                context->rxBufIdx = numToCopy;

                /* Check if all requested data has been read from the ring buffer */
                if (0ul == size)
                {
                    /* Enable the RX-error interrupt sources to update the error status */
                    Cy_SCB_SetRxInterruptMask(base, CY_SCB_UART_RECEIVE_ERR);

                    /* Call a completion callback if there was no abort receive called
                    * in the interrupt. The abort clears the number of the received bytes.
                    */
                    if (context->rxBufIdx > 0ul)
                    {
                        if (NULL != context->cbEvents)
                        {
                            context->cbEvents(CY_SCB_UART_RECEIVE_DONE_EVENT);
                        }
                    }

                    /* Continue receiving data in the ring buffer */
                    Cy_SCB_SetRxInterruptMask(base, CY_SCB_RX_INTR_LEVEL);
                }
                else
                {
                    tmpBuf = &tmpBuf[(byteMode) ? (numToCopy) : (2ul * numToCopy)];
                }
            }
        }

        /* Set up a direct RX FIFO receive */
        if (size > 0ul)
        {
            uint32_t halfFifoSize = Cy_SCB_GetFifoSize(base) / 2ul;

            /* Set up context */
            context->rxStatus  = CY_SCB_UART_RECEIVE_ACTIVE;

            context->rxBuf     = (void *) tmpBuf;
            context->rxBufSize = size;
            context->rxBufIdx =  numToCopy;

            /* Set the RX FIFO level to the trigger interrupt */
            Cy_SCB_SetRxFifoLevel(base, (size > halfFifoSize) ? (halfFifoSize - 1ul) : (size - 1ul));

            /* Enable the RX interrupt sources to continue data reading */
            Cy_SCB_SetRxInterruptMask(base, CY_SCB_UART_RX_INTR);
        }

        retStatus = CY_SCB_UART_SUCCESS;
    }

    return (retStatus);
}
```

这段代码实现了接收 UART（通用异步收发传输器）数据的功能。以下是对代码详细作用的描述：

1. **函数声明和初始化**：
   ```c
   cy_en_scb_uart_status_t Cy_SCB_UART_Receive(volatile stc_SCB_t *base, void *rxBuf, uint32_t size, cy_stc_scb_uart_context_t *context)
   ```
   该函数用于接收 UART 数据，返回类型是 `cy_en_scb_uart_status_t`，参数包括 UART 基地址 `base`，接收缓冲区 `rxBuf`，接收数据大小 `size` 和 UART 上下文 `context`。

2. **参数和上下文检查**：
   ```c
   #if !defined(NDEBUG)
   CY_ASSERT(NULL != context);
   CY_ASSERT(CY_SCB_UART_INIT_KEY == context->initKey);
   #endif /* !(NDEBUG) */
   ```
   在非调试模式下，检查上下文指针是否为 NULL 以及初始化密钥是否正确。

3. **检查是否有活动的传输请求**：
   ```c
   if (0ul == (context->rxStatus & CY_SCB_UART_RECEIVE_ACTIVE))
   ```
   如果没有活动的传输请求，则继续执行接收操作。

4. **临时变量和缓冲区设置**：
   ```c
   uint8_t  *tmpBuf = (uint8_t *) rxBuf;
   uint32_t numToCopy = 0ul;
   Cy_SCB_SetRxInterruptMask(base, CY_SCB_CLEAR_ALL_INTR_SRC);
   ```
   定义临时变量并禁用接收中断以停止环形缓冲区更新。

5. **从环形缓冲区获取数据**：
   ```c
   if (NULL != context->rxRingBuf)
   {
       numToCopy = Cy_SCB_UART_GetNumInRingBuffer(base, context);
       if (numToCopy > 0ul)
       {
           // 从环形缓冲区复制数据到接收缓冲区
           // 更新环形缓冲区尾部指针
       }
   }
   ```
   如果环形缓冲区存在，从中获取数据并复制到接收缓冲区。

6. **直接从 RX FIFO 接收数据**：
   ```c
   if (size > 0ul)
   {
       uint32_t halfFifoSize = Cy_SCB_GetFifoSize(base) / 2ul;
       context->rxStatus  = CY_SCB_UART_RECEIVE_ACTIVE;
       context->rxBuf     = (void *) tmpBuf;
       context->rxBufSize = size;
       context->rxBufIdx =  numToCopy;
       Cy_SCB_SetRxFifoLevel(base, (size > halfFifoSize) ? (halfFifoSize - 1ul) : (size - 1ul));
       Cy_SCB_SetRxInterruptMask(base, CY_SCB_UART_RX_INTR);
   }
   ```
   如果需要接收的数据大小大于 0，直接从 RX FIFO 中接收数据，并设置 RX FIFO 水平和中断。

7. **返回状态**：
   
   ```c
   return (retStatus);
   ```
   返回接收状态。

总结：
- 该函数先从环形缓冲区获取数据，然后设置直接从 RX FIFO 接收剩余的数据。
- 它会更新接收缓冲区和上下文信息，并处理中断和错误状态。
- 最终返回接收操作的状态。

这段代码确保了 UART 数据的高效接收，并处理可能出现的各种情况，如环形缓冲区存在数据和直接从 RX FIFO 接收数据。

> ### Rx FIFO 和环形缓冲区
>
> #### Rx FIFO
>
> **Rx FIFO（接收先进先出缓冲器）** 是一种在串行通信接口（如UART）中常见的硬件缓冲区，其主要作用和特点如下：
>
> 1. **数据缓冲**：Rx FIFO 用于暂时存储从串行通信线路接收到的数据字节。这种缓冲机制有助于管理发送设备和接收CPU之间的数据处理速度差异。
>
> 2. **先进先出**：数据按照接收顺序进行处理，先接收到的字节先被CPU读取。
>
> 3. **中断生成**：当接收到一定数量的数据（基于FIFO阈值）时，Rx FIFO 可以生成中断通知CPU处理数据，从而减少了对CPU的持续轮询需求。
>
> 4. **减轻CPU负担**：通过缓冲数据，Rx FIFO 减少了CPU中断次数，使CPU能够更高效地执行其他任务。
>
> #### 环形缓冲区
>
> **环形缓冲区（Ring Buffer）** 是一种常见的软件数据结构，用于管理数据流。其主要作用和特点包括：
>
> 1. **循环缓冲**：环形缓冲区以循环方式操作，缓冲区末尾与起始位置相连。这种机制允许高效地利用内存空间，而无需移动数据。
>
> 2. **数据存储与管理**：用于连续数据流的缓冲管理，非常适合在缓冲区满后重新覆盖最旧数据的场景。
>
> 3. **生产者-消费者模型**：通常用于一个系统部分（生产者）生成数据，另一个部分（消费者）处理数据的场景。环形缓冲区帮助管理生产者与消费者之间的同步，确保数据流的平滑。
>
> 4. **软件实现**：与Rx FIFO这种硬件特性不同，环形缓冲区通常在软件中实现，提供更大的灵活性，但需要适当管理以避免数据损坏。
>
> ### 作用对比
>
> - **Rx FIFO** 是特定于串行通信接口的硬件特性，通过最小化中断来缓冲临时数据和减轻CPU负担。
> - **环形缓冲区** 是用于软件中的通用数据结构，用于高效且循环地管理连续的数据流，适用于需要高效和循环数据管理的场景。
>
> 在实际应用中，两种机制可以结合使用：Rx FIFO 可以在硬件级别缓冲接收的串行数据，而环形缓冲区可以在软件级别更高效地管理数据流，并促进系统不同部分之间的交互。

### 1.2 数据发送

同样是周期发送或者中断触发后发送

**UART_SendData**

```c
//UART&ORIN
#define User_USB_SCB8_TYPE                         SCB8
#define User_USB_SCB8_UART_RX_PORT                 GPIO_PRT12
#define User_USB_SCB8_UART_RX_PIN                  0
#define User_USB_SCB8_UART_RX_MUX                  P12_0_SCB8_UART_RX
#define User_USB_SCB8_UART_TX_PORT                 GPIO_PRT12
#define User_USB_SCB8_UART_TX_PIN                  1
#define User_USB_SCB8_UART_TX_MUX                  P12_1_SCB8_UART_TX
#define User_USB_SCB8_UART_PCLK                    PCLK_SCB8_CLOCK
#define User_USB_SCB8_UART_IRQN                    scb_8_interrupt_IRQn

cy_stc_scb_uart_context_t   g_stc_uart8_context;

void UART_SendData( uint8_t *data, size_t size)
{
    // 检查输入参数
    if (data == NULL || size == 0)
    {
        return ;
    }
    // 调用Cypress的发送函数
    Cy_SCB_UART_Transmit(User_USB_SCB8_TYPE, data, size, &g_stc_uart8_context);
    StartSystemTimer(50000ul);
}
```

**Cy_SCB_UART_Transmit**

```c
cy_en_scb_uart_status_t Cy_SCB_UART_Transmit(volatile stc_SCB_t *base, void *txBuf, uint32_t size, cy_stc_scb_uart_context_t *context)
{
    cy_en_scb_uart_status_t retStatus = CY_SCB_UART_TRANSMIT_BUSY;

#if !defined(NDEBUG)
    /* Check that the initialization key was set before using the context */
    CY_ASSERT(NULL != context);
    CY_ASSERT(CY_SCB_UART_INIT_KEY == context->initKey);
#endif /* !(NDEBUG) */

    /* Check if there are no active transfer requests */
    if (0ul == (CY_SCB_UART_TRANSMIT_ACTIVE & context->txStatus))
    {
        /* Set up context */
        context->txStatus  = CY_SCB_UART_TRANSMIT_ACTIVE;

        context->txBuf     = txBuf;
        context->txBufSize = size;

        /* Set the level in TX FIFO to start a transfer */
        Cy_SCB_SetTxFifoLevel(base, (Cy_SCB_GetFifoSize(base) / 2ul));

        /* Enable the interrupt sources */
        if (CY_SCB_UART_SMARTCARD == base->unUART_CTRL.stcField.u2MODE)
        {
            /* Transfer data into TX FIFO and track SmartCard-specific errors */
            Cy_SCB_SetTxInterruptMask(base, CY_SCB_UART_TX_INTR);
        }
        else
        {
            /* Transfer data into TX FIFO */
            Cy_SCB_SetTxInterruptMask(base, CY_SCB_TX_INTR_LEVEL);
        }

        retStatus = CY_SCB_UART_SUCCESS;
    }

    return (retStatus);
}
```

### `Cy_SCB_UART_Transmit` 函数的详细描述

`Cy_SCB_UART_Transmit` 函数负责在使用赛普拉斯的串行通信模块（SCB）API的 PSoC（可编程系统芯片）环境中启动 UART 传输。以下是其组成部分和功能的详细说明：

#### 函数签名
```c
cy_en_scb_uart_status_t Cy_SCB_UART_Transmit(volatile stc_SCB_t *base, void *txBuf, uint32_t size, cy_stc_scb_uart_context_t *context)
```

- **参数**：
  - `base`：指向将处理 UART 传输的 SCB 实例的指针。
  - `txBuf`：指向包含要传输的数据的缓冲区的指针。
  - `size`：要传输的数据的大小（以字节为单位）。
  - `context`：指向 UART 上下文结构的指针，该结构保存 UART 操作的状态和配置。

- **返回类型**：
  - `cy_en_scb_uart_status_t`：指示 UART 传输请求状态的枚举类型。

#### 初始化检查和键断言
```c
#if !defined(NDEBUG)
    CY_ASSERT(NULL != context);
    CY_ASSERT(CY_SCB_UART_INIT_KEY == context->initKey);
#endif /* !(NDEBUG) */
```
- 此部分确保 `context` 不是 `NULL`，并且 `context` 中的初始化键已正确设置。这主要用于调试目的，以捕获设置中的错误。

#### 活动传输检查
```c
if (0ul == (CY_SCB_UART_TRANSMIT_ACTIVE & context->txStatus))
```
- 该条件检查是否存在活动的 UART 传输。如果没有活动传输（`txStatus` 未指示 `CY_SCB_UART_TRANSMIT_ACTIVE`），函数继续设置新的传输。

#### 上下文设置
```c
context->txStatus  = CY_SCB_UART_TRANSMIT_ACTIVE;
context->txBuf     = txBuf;
context->txBufSize = size;
```
- 将 `txStatus` 设置为指示活动传输。
- 在上下文结构中设置传输缓冲区（`txBuf`）及其大小（`txBufSize`）。

#### FIFO 级别配置
```c
Cy_SCB_SetTxFifoLevel(base, (Cy_SCB_GetFifoSize(base) / 2ul));
```
- 将传输 FIFO（先进先出）级别设置为 FIFO 大小的一半以启动传输。这有助于在正确的时刻触发 UART 传输。

#### 中断配置
```c
if (CY_SCB_UART_SMARTCARD == base->unUART_CTRL.stcField.u2MODE)
{
    Cy_SCB_SetTxInterruptMask(base, CY_SCB_UART_TX_INTR);
}
else
{
    Cy_SCB_SetTxInterruptMask(base, CY_SCB_TX_INTR_LEVEL);
}
```
- 根据 UART 的模式（普通或智能卡），启用适当的中断源。
  - 对于智能卡模式，启用与智能卡操作相关的特定中断。
  - 对于其他模式，启用基于级别的传输中断。

#### 返回状态
```c
retStatus = CY_SCB_UART_SUCCESS;
```
- 如果函数成功设置传输，它会将返回状态更新为 `CY_SCB_UART_SUCCESS`。

#### 函数返回
```c
return (retStatus);
```
- 最后，函数返回 UART 传输请求的状态。

### 总结
`Cy_SCB_UART_Transmit` 函数检查是否有正在进行的 UART 传输。如果没有，它配置上下文，设置 FIFO 级别，并根据 UART 模式启用必要的中断。函数然后返回操作的状态。这个函数对于以高效和中断驱动的方式管理 UART 传输至关重要，确保数据通过 UART 接口正确发送。

## Cy_SCB_UART_Transmit和Cy_SCB_UART_PutArray谁更适合持续稳定发送数据

在持续稳定地发送数据时，`Cy_SCB_UART_Transmit` 和 `Cy_SCB_UART_PutArray` 这两个函数都有其适用的场景。以下是它们的详细比较和适用情况：

### `Cy_SCB_UART_Transmit`

- **功能**：用于通过 UART 发送单个字节的数据。
- **用法**：通常用于发送单个字符或数据字节，适合逐字节发送数据。
- **适用场景**：适用于需要逐字节控制的情况，比如动态数据的发送或流控制。对于需要高频率发送的单字节数据，它可以提供更高的灵活性。

### `Cy_SCB_UART_PutArray`

- **功能**：用于通过 UART 发送一个字节数组。
- **用法**：一次性发送一整块数据数组，适合批量传输数据。
- **适用场景**：适用于需要发送较大数据块的情况，例如发送缓冲区中的数据。它通常在发送多个连续字节时更高效，因为减少了函数调用的开销，并可能减少了 UART 中断处理的次数。

### 比较与选择

1. **性能**：
   - `Cy_SCB_UART_PutArray` 一般来说更高效，尤其是当需要发送大量数据时，因为它一次性传输整个数组，减少了函数调用的次数和上下文切换的开销。
   - `Cy_SCB_UART_Transmit` 更适合逐字节发送，适合那些需要精确控制每个字节发送的场景。

2. **稳定性**：
   - 如果数据是连续的块（如文件数据或大缓冲区），使用 `Cy_SCB_UART_PutArray` 可以提高发送稳定性，因为它减少了中断处理次数和数据准备时间。
   - 如果需要逐步发送数据，或者数据发送的时序很重要，那么使用 `Cy_SCB_UART_Transmit` 可以提供更大的灵活性。

### 结论

对于持续稳定地发送数据，尤其是大量连续的数据块，`Cy_SCB_UART_PutArray` 通常是更合适的选择，因为它能够更高效地传输大块数据，减少了每次调用的开销，从而提高了发送的稳定性和效率。如果发送的数据量较小或者需要逐字节控制，可以选择 `Cy_SCB_UART_Transmit`。