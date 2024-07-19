# 代码走读-CM0核OTA升级逻辑及核心函数梳理

## SystemInit

这段代码是一个系统初始化函数 `SystemInit`，它通常在嵌入式系统启动时执行，以设置系统时钟、内存等待状态、禁用看门狗计时器（WDT）等。具体作用如下：

1. **禁用看门狗计时器（WDT）**：
    
    ```c
    #if defined(CY_SYSTEM_WDT_DISABLE)
        Cy_WDT_Disable();
    #endif
    ```
    
2. **设置电源电压调节**：
    ```c
    #if (CY_USE_PSVP == 0u)
        #if (CY_SYS_VCCD_SOURCE == CY_SYS_VCCD_PASS_TR)
            Cy_Power_SwitchToTransistor(CY_SYSREGHC_VADJ_1V150);
        #endif
    ```

3. **配置 ROM、RAM 和 Flash 的等待状态**：
    ```c
        CPUSS->unROM_CTL.stcField.u2SLOW_WS = 1u;
        CPUSS->unROM_CTL.stcField.u2FAST_WS = 0u;
        CPUSS->unRAM0_CTL0.stcField.u2SLOW_WS = 1u;
        CPUSS->unRAM0_CTL0.stcField.u2FAST_WS = 0u;
        ...
        FLASHC->unFLASH_CTL.stcField.u4WS = 1u;
    ```

4. **设置低频时钟源（LFCLK）**：
    ```c
        SRSS->unCLK_SELECT.stcField.u3LFCLK_SEL = CY_SYSCLK_LFCLK_IN_ILO0;
    ```

5. **启用和配置外部晶振（ECO）**：
    ```c
    #if CY_SYSTEM_USE_CLOCK == CY_SYSTEM_USE_ECO
        SRSS->unCLK_ECO_CONFIG.stcField.u1ECO_EN = 1ul;
        while(SRSS->unCLK_ECO_STATUS.stcField.u1ECO_OK == 0ul);
        while(SRSS->unCLK_ECO_STATUS.stcField.u1ECO_READY == 0ul);
    #endif
    ```

6. **启用和配置晶振（WCO）**：
    ```c
    #if defined(CY_SYSTEM_WCO_ENABLE)
        BACKUP->unCTL.stcField.u1WCO_EN = 1ul;
        while(BACKUP->unSTATUS.stcField.u1WCO_OK == 0ul);
    #endif
    ```

7. **设置 CPUSS 时钟分频器**：
    ```c
        CPUSS->unMEM_CLOCK_CTL.stcField.u8INT_DIV = 0u;
        CPUSS->unSLOW_CLOCK_CTL.stcField.u8INT_DIV = 1u;
        CPUSS->unPERI_CLOCK_CTL.stcField.u8INT_DIV = 1u;
        CPUSS->unTRC_DBG_CLOCK_CTL.stcField.u8INT_DIV = 0u;
    ```

8. **配置并启用各个 PLL**：
    ```c
        SRSS->unCLK_PATH_SELECT[1].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;
        SRSS->CLK_PLL400M[0].unCONFIG.stcField.u1ENABLE = 1ul;
        ...
        while(SRSS->CLK_PLL400M[0].unSTATUS.stcField.u1LOCKED == 0ul);
        ...
    ```

9. **设置高频时钟源（HFCLK）**：
    ```c
        struct {cy_en_hf_clk_dividers_t targetDiv; cy_en_hf_clk_sources_t source;} clkHfSetting[SRSS_NUM_HFROOT] = 
        {
            { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE, .source = CY_SYSCLK_HFCLK_IN_CLKPATH3 },
            ...
        };
    ```

10. **启用并配置 ILO0 时钟**：
    ```c
        Cy_WDT_Unlock();
        SRSS->unCLK_ILO0_CONFIG.stcField.u1ENABLE = 1;
        SRSS->unCLK_ILO0_CONFIG.stcField.u1ILO0_BACKUP = 1ul;
        Cy_WDT_Lock();
    ```

11. **更新系统核心时钟**：
    ```c
        SystemCoreClockUpdate();
    ```

这段代码主要用于配置系统的时钟源和分频器，设置不同内存的等待状态，以及启用必要的晶振和 PLL，以确保系统在启动时能够以正确的时钟和电源配置运行。这是嵌入式系统初始化过程中非常关键的一部分。

## FlashInit

这段代码的作用是初始化闪存控制器 (Flash Controller) 以便系统能够正确地使用闪存。以下是对每一行代码的具体描述：

1. `Cy_Flashc_MainWriteEnable();`：启用主闪存区 (Main Flash) 的写操作权限。通常情况下，主闪存区的写操作是被保护的，需要明确地启用才能进行写操作。

2. `Cy_Flashc_WorkWriteEnable();`：启用工作闪存区 (Work Flash) 的写操作权限。类似于主闪存区，工作闪存区的写操作也需要明确地启用。

3. `Cy_Flashc_SetMainBankMode(CY_FLASH_DUAL_BANK_MODE);`：将主闪存区设置为双银行模式 (Dual Bank Mode)。双银行模式允许闪存的不同部分同时进行读写操作，提高了系统的并发处理能力。

4. `Cy_Flashc_CM0_CacheDisable();`：禁用 CM0 (Cortex-M0) 内核的缓存。这通常是在更新闪存内容前执行，以避免缓存中的旧数据影响到新的闪存内容。

5. `Cy_Flashc_InvalidateFlashCacheBuffer();`：使闪存缓存失效，清除缓存中的所有数据。这是为了确保接下来的读操作能从闪存中读取最新的数据，而不是从缓存中读取旧的数据。

这段代码主要用于**配置和初始化闪存控制器，使系统可以安全、高效地进行闪存读写操作**。由于当前的启动 ROM 代码尚不支持这些设置，所以这些初始化代码被暂时放在这里执行。

## FlashSwitchFlow

这段代码的作用是执行闪存切换操作（Flash Switch Flow），以确保系统在闪存块检测后能够根据特定条件进行相应的切换操作。以下是对每一部分代码的详细描述：

1. `uint32_t status = Cy_Flash_BlankCheck(&sromContext1, &blankCheckConfig, CY_FLASH_DRIVER_BLOCKING);`：
   - 调用 `Cy_Flash_BlankCheck` 函数对指定的闪存区域进行空白检查，判断该区域是否未被编程（即是否为空白）。该函数返回一个状态值 `status`，表明检查结果。
   - 参数 `sromContext1` 和 `blankCheckConfig` 分别是 SROM 上下文和空白检查配置结构，`CY_FLASH_DRIVER_BLOCKING` 表示该操作为阻塞操作，等待完成后才返回。

2. `if(status != CY_FLASH_DRV_SUCCESS)`：
   - 检查空白检查操作的状态。如果返回值不是 `CY_FLASH_DRV_SUCCESS`（表示成功），则表示该闪存区域未通过空白检查，进入错误处理流程。

3. `if(VAL_MAKER_ON_WORK == 0xBBBBBBBB)`：
   - 检查 `VAL_MAKER_ON_WORK` 的值是否为 `0xBBBBBBBB`。如果是，则调用 `SwitchMapB_ThenRestart()` 函数。

4. `SwitchMapB_ThenRestart();`：
   - 切换到 B 闪存映射，然后重启系统。该函数的具体实现未给出，但可以推测其作用是切换闪存映射并重启。

5. `else if (VAL_MAKER_ON_WORK == 0xAAAAAAAA)`：
   - 检查 `VAL_MAKER_ON_WORK` 的值是否为 `0xAAAAAAAA`。如果是，则不执行任何操作，继续保持当前状态。

6. `else`：
   - 如果 `VAL_MAKER_ON_WORK` 的值不是上述两种情况之一，进入 `while(1);` 死循环，表示遇到了非预期状态。这个死循环防止系统继续执行，避免潜在的错误传播。

7. `else`：
   - 如果 `status` 是 `CY_FLASH_DRV_SUCCESS`（表示空白检查通过），不执行任何操作，继续保持当前状态。

总结起来，这段代码的作用是：
- 对指定的闪存区域进行空白检查。
- 根据空白检查结果和特定标志值 `VAL_MAKER_ON_WORK` 的值，决定是否切换闪存映射。
- 当检查结果不通过且标志值为 `0xBBBBBBBB` 时，执行闪存映射切换并重启系统。
- 在其他情况下，或保持当前状态，或进入死循环以防止错误继续传播。

### SwitchMapB_ThenRestart

这段代码定义了一个名为 `SwitchMapB_ThenRestart` 的静态函数，其主要作用是将闪存的映射切换到映射B，并重启系统。以下是对每一行代码的详细描述：

1. `FLASHC->unFLASH_CTL.stcField.u1MAIN_MAP = CY_FLASH_MAPPING_B;`:
   - 设置闪存控制寄存器中的主映射位 (`MAIN_MAP`) 为映射B (`CY_FLASH_MAPPING_B`)。

2. `FLASHC->unFLASH_CTL.u32Register;`:
   - 读取闪存控制寄存器的值。虽然该行代码并未显式执行操作，但它可能用于确保先前的写操作已被处理。

3. `__ISB();`:
   - 执行指令同步屏障（Instruction Synchronization Barrier）指令，确保所有先前的指令已执行完毕，并刷新指令流水线，以确保后续指令能正确执行。

4. `FLASHC->unFLASH_CMD.stcField.u1INV = 0x1u;`:
   - 设置闪存命令寄存器中的无效位 (`INV`) 为1，表示请求无效化闪存缓存。

5. `while(FLASHC->unFLASH_CMD.stcField.u1INV == 0x1u);`:
   - 等待无效位 (`INV`) 清零，表示闪存缓存已成功无效化。

6. `//tztek_power_down();`:
   - 这是一个注释，指向一个可能的电源关闭函数调用。

7. `//NVIC_SystemReset();`:
   - 这是一个注释，指向一个系统复位函数调用，可能用于通过NVIC执行系统复位。

8. `//Cy_SysTick_DelayInUs(5000000ul);`:
   - 这是一个注释，指向一个可能的延时函数调用，用于延迟指定的时间（5000000微秒）。

9. `BACK_TO_START();`:
   - 调用前述定义的 `BACK_TO_START` 函数，重新启动系统。

10. `return;`:
    - 函数返回，无返回值。

总结起来，这段代码的作用是：
1. 切换闪存的映射到映射B。
2. 执行指令同步屏障，确保所有指令按预期执行。
3. 请求并等待无效化闪存缓存。
4. 通过调用 `BACK_TO_START` 函数重新启动系统。

注释掉的代码行（如 `tztek_power_down`、`NVIC_SystemReset`、`Cy_SysTick_DelayInUs`）提供了其他可能的系统复位或延迟机制，但在当前函数实现中未被激活。

#### BACK_TO_START

```assembly
BACK_TO_START:
        DSB     ; Make sure outstanding transfers are done        
        ISB        ; Make sure outstanding transfers are done
        LDR R0, =0x10000000
        LDR R1, [R0] ; Load addr 0x1000 0000 contet to R1
        MSR MSP, R1
        LDR R1, [R0, #4]
        BLX  R1
 END
```

这段代码是用汇编语言编写的ARM Cortex-M处理器启动代码的一部分。其主要作用是初始化堆栈指针并跳转到复位向量（Reset Vector）指定的地址，以启动固件执行。以下是对每一行代码的详细描述：

1. `BACK_TO_START:`:
   - 标签 `BACK_TO_START`，用作程序执行的起始点或跳转目标。

2. `DSB`:
   - Data Synchronization Barrier（数据同步屏障）指令，用于确保所有先前的数据传输完成，确保内存操作的顺序。

3. `ISB`:
   - Instruction Synchronization Barrier（指令同步屏障）指令，用于确保所有先前的指令执行完成，确保后续指令能够正确执行。

4. `LDR R0, =0x10000000`:
   - 将立即数 `0x10000000` 加载到寄存器 `R0` 中。该地址通常是向量表（Vector Table）起始地址。

5. `LDR R1, [R0]`:
   - 从 `R0` 指向的内存地址 `0x10000000` 处加载数据到寄存器 `R1` 中。该地址通常存储堆栈指针初始值。

6. `MSR MSP, R1`:
   - 将寄存器 `R1` 的值写入主堆栈指针（Main Stack Pointer, MSP）。这一步将堆栈指针初始化为向量表中的值。

7. `LDR R1, [R0, #4]`:
   - 从 `R0` 指向的内存地址 `0x10000004` 处加载数据到寄存器 `R1` 中。该地址通常存储复位向量（Reset Vector）的地址。

8. `BLX R1`:
   - 分支并链接到寄存器 `R1` 指定的地址，即跳转到复位向量指定的地址开始执行固件代码。

9. `END`:
   - 标签 `END`，通常表示代码的结束点或跳转目标（这里可以忽略）。

总结起来，这段代码的作用是：
1. 确保所有数据和指令传输完成。
2. 从向量表加载初始堆栈指针值，并将其设置为主堆栈指针。
3. 从向量表加载复位向量地址，并跳转到该地址开始执行固件代码。

## MyInit

这段代码定义了一个名为 `MyInit` 的函数，用于初始化某些硬件组件（例如UART）。具体描述如下：

1. `UART_Init_With_Interrupt(115200ul, 80000000ul);`
   - 以波特率115200和时钟频率80000000初始化UART并启用中断。

2. `UART8_Init_With_Interrupt(115200ul, 80000000ul);`
   - 以波特率115200和时钟频率80000000初始化UART8并启用中断。

接下来是一组被注释掉的初始化函数和调试打印语句：

3. `// iic_initall();`
   - 注释掉的代码，用于初始化I2C接口。

4. `// Term_Printf("*********************iic_initall*************************\r\n");`
   - 注释掉的代码，打印I2C初始化完成的调试信息。

5. `// Cy_SysTick_DelayInUs(10000ul);`
   - 注释掉的代码，延迟10000微秒（10毫秒）。

6. `// ina3221_initall();`
   - 注释掉的代码，用于初始化INA3221（一个电流/电压监控芯片）。

7. `// Term_Printf("*********************ina3221_initall*************************\r\n");`
   - 注释掉的代码，打印INA3221初始化完成的调试信息。

8. `// Cy_SysTick_DelayInUs(10000ul);`
   - 注释掉的代码，延迟10000微秒（10毫秒）。

9. `// tztek_aht2415c_initALL();`
   - 注释掉的代码，用于初始化TZTEK AHT2415C传感器。

10. `// tztek_lm75bdp_initall();`
    - 注释掉的代码，用于初始化TZTEK LM75BDP温度传感器。

11. `// Timer_Init();`
    - 注释掉的代码，用于初始化定时器。

总结：

`MyInit` 函数的当前实现初始化了两个UART接口并启用了中断。注释掉的部分代码指向了其他硬件组件的初始化（如I2C接口、INA3221监控芯片、TZTEK传感器和定时器），以及相关的延迟和调试打印语句。如果这些注释被取消，函数将会初始化更多的硬件组件，并提供相应的调试信息。

## Orin_power_on

这段代码定义了一个名为 `Orin_power_on` 的函数，主要用于配置和初始化多个GPIO引脚，并通过设置这些引脚来上电或控制各种设备组件。详细描述如下：

1. **配置输出引脚**：
   - 创建一个名为 `user_led_port_pin_cfg` 的结构体实例，用于配置GPIO引脚的参数。
   - 设置引脚的驱动模式为 `CY_GPIO_DM_STRONG_IN_OFF`，表示强驱动模式，且输入被禁用。

2. **初始化各个引脚并设置多路复用（MUX）**：
   - 初始化多个引脚，分别对应不同的硬件组件，如风扇（FAN）、电源（ORIN、NANO、PCLE等）、LED等。
   - 每个引脚的多路复用（MUX）配置为相应的宏定义值（如 `USER_FAN1_PIN_MUX`）。

3. **上电流程**：
   - 使用 `Cy_GPIO_Set` 函数来设置（上电）多个引脚，这些引脚对应不同的硬件组件。具体上电顺序如下：
     - 上电ORIN12V和ORIN5V。
     - 延迟100000微秒（0.1秒）。
     - 上电NANO5V。
     - 延迟100000微秒（0.1秒）。
     - 上电PCLE5V、CAM12V、AENR、MM12V、88383V3和LED3V3。
     - 延迟100000微秒（0.1秒）。
     - 上电5G3V8和ETH5V，并清除SWITCHORINMCU引脚。
     - 设置88E6390RESET引脚，延迟100000微秒（0.1秒），然后清除88E6390RESET引脚。
     - 延迟100000微秒（0.1秒），设置Q2112RESET引脚。
     - 延迟100000微秒（0.1秒），设置88E1512RESET引脚。
     - 上电FAN1和FAN2。
     - 最后延迟50000微秒（0.05秒）。

### 总结：
`Orin_power_on` 函数的主要作用是通过配置和设置多个GPIO引脚来上电和控制各种硬件组件。这些组件包括风扇、电源、LED、以太网等。函数通过一系列的延迟和引脚设置确保每个组件在正确的时间上电或复位，从而实现系统的正常启动。

## MainMenu

这段代码定义了一个名为 `MainMenu` 的函数，它用于处理从UART接收到的输入并根据不同的输入执行不同的操作。这是一个简化的菜单系统，用于管理闪存操作和系统控制。以下是详细描述：

### 变量声明：
1. **`uint8_t u8ReadBuf`**：
   - 用于存储从UART接收到的数据。

2. **`uint8_t action = 0`**：
   - 用于跟踪用户选择的操作，初始值为0，表示没有操作被选择。

### 打印菜单（注释掉的代码）：
- 这些注释掉的代码用于向用户显示菜单选项，但当前被注释掉了。

### 菜单操作循环：
- 使用 `while (action == 0)` 循环，确保在接收到有效的用户输入之前持续等待。

#### 接收用户输入：
- 使用 `Cy_SCB_UART_GetArray` 从UART接收一个字节的数据存储在 `u8ReadBuf` 中。
- 这个函数是非阻塞的，如果没有接收到数据，会一直等待。

#### 处理用户输入：
- 使用 `switch` 语句，根据 `u8ReadBuf` 的值执行不同的操作。

1. **`case '1'`**：
   - 调用 `EraseCodeFlashLogicalArea1()` 擦除代码闪存的逻辑区域1。
   - 调用 `Main_MenuInternalFlash_receive(0)` 接收文件并在 `MAPB_CM0p` 上编程闪存。
   - 将 `action` 设为1，表示已处理此选项。

2. **`case '2'`**：
   - 调用 `EraseCodeFlashLogicalArea2()` 擦除代码闪存的逻辑区域2。
   - 调用 `Main_MenuInternalFlash_receive(1)` 接收文件并在 `MAPB_CM7_0` 上编程闪存。
   - 将 `action` 设为2，表示已处理此选项。

3. **`case '3'`**：
   - 调用 `Main_MenuInternalFlash_receive(2)` 接收文件并在 `MAPB_CM7_1` 上编程闪存。
   - 将 `action` 设为3，表示已处理此选项。

4. **`case '4'`**：
   - 调用 `EraseSectorWorkSectors_SA0()` 擦除工作闪存的扇区。
   - 调用 `ProgramRow_B()` 对B区域进行编程。
   - 使用 `Term_Printf` 打印 "SWITCH WORK FLASH B!!" 信息。
   - 禁用中断，清除并禁用指定中断。
   - 设置 `action` 为4。
   - 延迟5秒，然后调用 `BACK_TO_START()` 重启系统。

5. **`case '5'`**：
   - 打印 "Exiting OTA SYSTEM." 信息。
   - 调用 `Orin_power_down()` 关闭系统。
   - 延迟3秒，打印 "Exiting OTA SYSTEM2." 信息。
   - 启用CM7_0核心。
   - 延迟300秒，然后返回以退出菜单。

6. **`default`**：
   - 如果接收到的输入不是上述选项，回显输入到UART。

### 总结：
`MainMenu` 函数提供了一个简单的菜单系统，用户可以通过UART输入选择不同的操作，如闪存编程、工作闪存切换和系统关闭。函数通过接收用户输入、执行相应的操作并回显未识别的输入，实现了一个基本的交互式系统管理界面。