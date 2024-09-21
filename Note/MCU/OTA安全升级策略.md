# OTA安全升级策略

- [OTA安全升级策略](#ota安全升级策略)
  - [1 芯片结构](#1-芯片结构)
  - [2 OTA流程](#2-ota流程)
  - [3 系统模块](#3-系统模块)
    - [3.1 双区支持](#31-双区支持)
    - [3.2 Bootloader](#32-bootloader)
    - [3.3 烧录](#33-烧录)
    - [3.4 分区切换](#34-分区切换)
    - [**3.5 业务核挂死可恢复**](#35-业务核挂死可恢复)
    - [\***3.6 OTA超时退出**](#36-ota超时退出)
  - [4 总结思考](#4-总结思考)
    - [4.1 启动时序](#41-启动时序)
    - [4.2 烧录地址](#42-烧录地址)
    - [4.3 多核的任务分工](#43-多核的任务分工)
  - [5 其他安全策略](#5-其他安全策略)
    - [**5.1 接收端UART命令重排**](#51-接收端uart命令重排)
    - [**5.2 UART数据持续稳定收发**](#52-uart数据持续稳定收发)
  - [6 一些未解决的问题](#6-一些未解决的问题)
    - [\*6.1 UART单通道同步OTA刷写与调试log回显失败率较高](#61-uart单通道同步ota刷写与调试log回显失败率较高)
- [会议纪要](#会议纪要)


## 1 芯片结构

- cm0、cm7_0和cm7_1三核间独立；

- MCU上电后先启动cm0核，然后cm0核中的代码会使能cm7_0和cm7_1；

- 控电和OTA代码都运行在cm0核，业务代码在cm7_0和cm7_1。


## 2 OTA流程

<img src="https://s2.loli.net/2024/08/13/k3XMNlZvucwqL87.png" style="zoom: 67%;" />

- 对区刷写

  <img src="https://s2.loli.net/2024/08/13/NuSACdYOWnxlqGF.png" style="zoom: 67%;" />

  <img src="https://s2.loli.net/2024/08/13/v6l7Pg5u9Vkhr3M.png" style="zoom:67%;" />

- 代码备份（新旧版本）

- BootLoader指定运行代码的起始地址

## 3 系统模块

### 3.1 双区支持

```c
Cy_Flashc_SetMainBankMode(CY_FLASH_DUAL_BANK_MODE);
```

### 3.2 Bootloader

运行区/Workflash检测 -> 切换目标分区

![](https://s2.loli.net/2024/08/13/X5fw9tAgzs8R63U.png)

### 3.3 烧录

<img src="https://s2.loli.net/2024/08/13/T52arY7S8Zz3mEp.png" style="zoom: 67%;" />

- 首次烧录（MCU双分区中都没有OTA程序）：需要IAR接调试器；
- 其他情况：可以远程升级。

### 3.4 分区切换

通过以下接口修改Workflash的值

```c
void ProgramRow_A(void)
{
    uint32_t status = CY_FLASH_DRV_SUCCESS;
    /* 2.1. Program flash whole sector    */
    for (uint32_t offsetAddr = 0; offsetAddr < CODE_FLASH_SECTOR_SIZE; offsetAddr += TEST_PROGRAM_SIZE) {
        programRowConfigSA0.destAddr = (uint32_t*)(WFLASH_ADDRESS + offsetAddr);
        programRowConfigSA0.dataAddr = writeData_A;
        status = Cy_Flash_ProgramRow(&FlashContext, &programRowConfigSA0, CY_FLASH_DRIVER_BLOCKING);
        CY_ASSERT(status == CY_FLASH_DRV_SUCCESS);       
    }
}
```

修改之后重启MCU，Bootloader就会按照Workflash的值完成切区。

### **3.5 业务核挂死可恢复**

- cm0使能业务核之后，始终保持与cm7_0的核间通信（cm7_0 -> cm0，每10秒一次将超时时间置为60秒）；
- 一旦核间通信停发，即认为业务核挂死，cm0核就会执行切换分区，回退版本。

### ***3.6 OTA超时退出**

收到OTA升级指令后，cm0通过接收cm7_0周期发来的核间通信数据确认业务核工作状态，确认正常后立即进入OTA模式，并在IPC中断中关闭业务核，cm7_0和cm7_1将不再运行；

进入OTA模式后，cm0将触发定时器开始计时，超出设定超时时间后将自动退出OTA模式，恢复使能业务核，cm7_0和cm7_1将重新启用运行。

1、进入OTA模式后开始计时，十分钟之内未退出就整机重启复位；
2、进入OTA模式后，每次刷写操作完成之后开始计时，三分钟之内未操作就整机重启复位。

```c
if (OTA_FLAG) {
    for (;;) {
        MainMenu();
        if (menuExited) {
            /* Enable CM7_0/1. */
            Cy_SysEnableApplCore(CORE_CM7_0, CY_CORTEX_M7_0_APPL_ADDR);
            Cy_SysEnableApplCore(CORE_CM7_1, CY_CORTEX_M7_1_APPL_ADDR);
            menuExited = 0;
            break;      
        }
    }
    OTA_FLAG = 0;
}
```

## 4 总结思考

### 4.1 启动时序

Flash初始化 -> Bootloader -> 业务核 （-> OTA -> 重启）。

### 4.2 烧录地址

被升级刷写的地址始终是`0x12000000`，切换分区时不过是将`0x10000000`的地址和前者重新做了互换映射。

### 4.3 多核的任务分工

- cm0主要负责：
  - Bootloader；
  - 上电；
  - OTA。
- cm7_0当前负责：
  - 整机、摄像头、毫米波雷达下电复位；
  - Orin、NX单控下电和上电（需要修改）；
  - 监听来自SOC的UART指令以进入OTA模式；
  - 向SOC周期上报监控数据：累计发送数据计数、当前版本和运行分区、温、湿度、电源状态（包括9路INA3221电流电压和17路ADC）。
- cm7_1当前无业务。

## 5 其他安全策略

### **5.1 接收端UART命令重排**

**问题背景**：UART命令传输过程中概率性丢失若干个字节，由于是队列缓存，前一帧的部分数据丢失会导致后续所有帧数据格式错序。

**问题分析**：原本认为UART中断接收和读取为异步操作，多次调试后发现，实际上UART中断触发依赖于读取，或者说，在调用读取函数Cy_SCB_UART_Receive之后才会触发中断Cy_SCB_UART_Interrupt，由此可认为原本接收读取的逻辑已是同步处理。所以该问题是串口通信过程中偶发性的错误，而非代码实现上的bug导致。

**解决方法**：基于部分数据丢失是偶发事件，由此可推断：连续两帧UART数据之间，出错未读到的位置是随机的，但因为稳定连续读取18位，所以一次读取的数据中总是包含了顺序正确的两段子串，将两段子串更换顺序之后重新组合即可得到正确的命令。截取处理示例如下：

```c
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$		// 正确指令正常识别
10s ---------->>>>>

*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
5s -------->>>>>
Millimeter-wave radar request for power down!!!!
5s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: 01060500000002$$^^
Rearranged command is: ^^01060500000002$$		// 错序但数据完整指令重排后能够识别（1）
10s ---------->>>>>

*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
5s -------->>>>>
Millimeter-wave radar request for power down!!!!
5s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: 01^^01060500000002
Rearranged command is: 01^^01060500000002		// 错序但数据不完整指令不能识别
Checksum mismatch. Expected: 0x00, Calculated: 0xED
10s ---------->>>>>

*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
5s -------->>>>>
5s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$		// 错序但数据完整指令重排后能够识别（2）
10s ---------->>>>>
```

原本UART读取指令时，依赖此前所有数据都要正确传递不容有失；

而应用该策略后，依赖条件变为：只要最近两条数据中有一条传递无失即可。提高了数据识别的安全稳定性，保证能在接收至多两次OTA命令后顺利识别并切换到OTA模式。

### **5.2 UART数据持续稳定收发**

**问题背景**：MCU通过UART_SendData向SOC发送数据持续一段时间之后，SOC串口中会收到一段乱码，之后就不会再显示新的UART数据（跑在SOC上的ros服务也不能收到），但是此时由SOC向MCU发送下电指令（SOC并未断过电），MCU是能接收并处理的（MCU没死，业务程序还在跑），怀疑是UART_SendData接口有问题：
1、163的设备是以10s为周期，持续向SOC发送长度为1000的字符串；
2、141的设备是以10s为周期，持续向SOC发送长度为700的字符串。

![](https://s2.loli.net/2024/08/12/R79OqvLDBEKCrGV.png)

**问题分析**：系统启动后，状态监控数据总能稳定发送一段时间，而发送数据长度不同时，开始出现乱码并停发数据的时间也不太一样，所以怀疑是资源紧张导致的UART通信阻塞（暂未定位到根本原因）。通过查看源码发现，负责发送UART数据的`UART_SendData`实际上调用的是`Cy_SCB_UART_Transmit`，而`Cy_SCB_UART_Transmit`更适用于发送单字节数据的场景，在持续发送如此大量的数据时可能会出现一些非预期问题。

另一方面，Cypress提供了一个更适合大量数据发送场景的接口`Cy_SCB_UART_PutArray`，相比于前者，它能减少中断的处理次数和上下文切换的开销，提高发送稳定性。

**解决方法**：更换接口：`Cy_SCB_UART_Transmit` -> `Cy_SCB_UART_PutArray`。

![](https://s2.loli.net/2024/08/13/8X1EZaTCFIHnpym.png)

更换接口后，持续测试发送稳定性（截图数据显示业务核已运行5728 * 10s ≈ 15.9h）。

MCU与SOC之间UART数据通信的稳定性是OTA安全升级的基础，电源状态数据的持续稳定收发直接决定和反映了当前UART信道的通畅状态，是OTA安全升级策略中的关键一环。

## 6 一些未解决的问题

### *6.1 UART单通道同步OTA刷写与调试log回显失败率较高

**问题描述**

在A样裸板MCU和SOC间只有单条UART通道的硬件环境下，测试同时进行OTA版本刷写与调试log回显任务，实测发现SOC侧可能是因为负载过高等问题导致刷写失败和log数据丢失（本质上刷写失败也是丢数据所致）。

**问题影响**

A样整机远程升级失败率较高；重复尝试有概率升级成功，但耗时较久。

**解决方法**

因为OTA刷写和log回显都是必要功能，所以新版硬件中不得不增加至两路UART，按7.29日会议结论，应当考虑扩展新的UART资源。此外，完成137、141整机MCU版本升级。


# 会议纪要

1. 波特率不是时钟频率整数倍导致的错位；
2. 开启奇偶校验提高准确性；
3. 观察错误中断是否触发看是否有数据丢失；
4. 前后两帧数据内容不同，导致语义解析错误；
5. 数据长度太长，定义好结构体缩减数据长度；
6. 频繁发送UART数据时加入delay；
7. 将ringBuffer策略修改为polling测试；
8. 升级脚本（SOC侧OTA上位机程序）完善，增加回显。
