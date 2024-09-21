

ADC电压信息输出对应变量排布

```
SOC_Printf("\rSOC1_VDD_3V3_ADC result = %0.2f V\r\n", g_adc_voltage_all[0]);   SOC_Printf("\rSOC1_VDD_1V8_ADC result = %0.2f V\r\n", g_adc_voltage_all[1]);   SOC_Printf("\rSOC2_VDD_3V3_ADC result = %0.2f V\r\n", g_adc_voltage_all[2]);   SOC_Printf("\rSOC2_VDD_1V8_ADC result = %0.2f V\r\n", g_adc_voltage_all[3]);
SOC_Printf("\rETH_VDD_3V3_ADC result = %0.2f V\r\n", g_adc_voltage_all[4]);
SOC_Printf("\rETH_VDD_1V8_ADC result = %0.2f V\r\n", g_adc_voltage_all[5]);
SOC_Printf("\rETH_VDD_1V5_ADC result = %0.2f V\r\n", g_adc_voltage_all[6]);
SOC_Printf("\rETH_VDD_1V1_ADC result = %0.2f V\r\n", g_adc_voltage_all[7]);
SOC_Printf("\rETH_VDD_0V88_ADC result = %0.2f V\r\n", g_adc_voltage_all[8]);
SOC_Printf("\rETH_VDD_1V2_ADC result = %0.2f V\r\n", g_adc_voltage_all[9]);
SOC_Printf("\r5G_VDD_3V8_ADC result = %0.2f V\r\n", g_adc_voltage_all[10]);
SOC_Printf("\rPCIE_VDD_1V8_ADC result = %0.2f V\r\n", g_adc_voltage_all[11]);
SOC_Printf("\rPCIE_VDD_3V3_ADC result = %0.2f V\r\n", g_adc_voltage_all[12]);
SOC_Printf("\rLED_VDD_3V3_M_ADC result = %0.2f V\r\n", g_adc_voltage_all[13]);
SOC_Printf("\rN20527916 result = %0.2f V\r\n", g_adc_voltage_all[14]);
SOC_Printf("\rN20527927 result = %0.2f V\r\n", g_adc_voltage_all[15]);
SOC_Printf("\rN20527938 result = %0.2f V\r\n", g_adc_voltage_all[16]);
```

替换宏定义`cpress-traveo_ii-bsp_freertos/tviibh8m/hdr/rev_c/bb_bsp_tviibh8m.h`

```c
#define CY_ADC_POT_MACRO1                        PASS0_SAR0
#define CY_ADC_POT_IN_NO1                   CY_ADC_PIN_ADDRESS_AN24
#define CY_ADC_POT_PORT1                         GPIO_PRT8
#define CY_ADC_POT_PIN1                          1
#define CY_ADC_POT_PIN_MUX1                      P8_1_GPIO
#define CY_ADC_POT_PCLK1                         PCLK_PASS0_CLOCK_SAR0
#define CY_ADC_POT_IRQN1                         pass_0_interrupts_sar_0_IRQn

#define CY_ADC_POT_MACRO2                        PASS0_SAR0
#define CY_ADC_POT_IN_NO2                   CY_ADC_PIN_ADDRESS_AN25
#define CY_ADC_POT_PORT2                         GPIO_PRT8
#define CY_ADC_POT_PIN2                          2
#define CY_ADC_POT_PIN_MUX2                      P8_2_GPIO
#define CY_ADC_POT_PCLK2                         PCLK_PASS0_CLOCK_SAR0
#define CY_ADC_POT_IRQN2                         pass_0_interrupts_sar_0_IRQn

#define CY_ADC_POT_MACRO3                        PASS0_SAR0
#define CY_ADC_POT_IN_NO3                   CY_ADC_PIN_ADDRESS_AN26
#define CY_ADC_POT_PORT3                         GPIO_PRT8
#define CY_ADC_POT_PIN3                          3
#define CY_ADC_POT_PIN_MUX3                      P8_3_GPIO
#define CY_ADC_POT_PCLK3                         PCLK_PASS0_CLOCK_SAR0
#define CY_ADC_POT_IRQN3                         pass_0_interrupts_sar_0_IRQn


#define CY_ADC_POT_MACRO4                        PASS0_SAR1
#define CY_ADC_POT_IN_NO4                   CY_ADC_PIN_ADDRESS_AN0
#define CY_ADC_POT_PORT4                         GPIO_PRT10
#define CY_ADC_POT_PIN4                          4
#define CY_ADC_POT_PIN_MUX4                      P10_4_GPIO
#define CY_ADC_POT_PCLK4                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN4                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO5                        PASS0_SAR1
#define CY_ADC_POT_IN_NO5                   CY_ADC_PIN_ADDRESS_AN1
#define CY_ADC_POT_PORT5                         GPIO_PRT10
#define CY_ADC_POT_PIN5                          5
#define CY_ADC_POT_PIN_MUX5                      P10_5_GPIO
#define CY_ADC_POT_PCLK5                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN5                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO6                        PASS0_SAR1
#define CY_ADC_POT_IN_NO6                   CY_ADC_PIN_ADDRESS_AN2
#define CY_ADC_POT_PORT6                         GPIO_PRT10
#define CY_ADC_POT_PIN6                          6
#define CY_ADC_POT_PIN_MUX6                      P10_6_GPIO
#define CY_ADC_POT_PCLK6                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN6                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO7                        PASS0_SAR1
#define CY_ADC_POT_IN_NO7                   CY_ADC_PIN_ADDRESS_AN3
#define CY_ADC_POT_PORT7                         GPIO_PRT10
#define CY_ADC_POT_PIN7                          7
#define CY_ADC_POT_PIN_MUX7                      P10_7_GPIO
#define CY_ADC_POT_PCLK7                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN7                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO8                        PASS0_SAR1
#define CY_ADC_POT_IN_NO8                   CY_ADC_PIN_ADDRESS_AN6
#define CY_ADC_POT_PORT8                         GPIO_PRT12
#define CY_ADC_POT_PIN8                          2
#define CY_ADC_POT_PIN_MUX8                      P12_2_GPIO
#define CY_ADC_POT_PCLK8                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN8                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO9                        PASS0_SAR1
#define CY_ADC_POT_IN_NO9                   CY_ADC_PIN_ADDRESS_AN7
#define CY_ADC_POT_PORT9                         GPIO_PRT12
#define CY_ADC_POT_PIN9                          3
#define CY_ADC_POT_PIN_MUX9                      P12_3_GPIO
#define CY_ADC_POT_PCLK9                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN9                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO10                        PASS0_SAR1
#define CY_ADC_POT_IN_NO10                   CY_ADC_PIN_ADDRESS_AN8
#define CY_ADC_POT_PORT10                         GPIO_PRT12
#define CY_ADC_POT_PIN10                          4
#define CY_ADC_POT_PIN_MUX10                      P12_4_GPIO
#define CY_ADC_POT_PCLK10                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN10                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO11                        PASS0_SAR1
#define CY_ADC_POT_IN_NO11                   CY_ADC_PIN_ADDRESS_AN9
#define CY_ADC_POT_PORT11                         GPIO_PRT12
#define CY_ADC_POT_PIN11                          5
#define CY_ADC_POT_PIN_MUX11                      P12_5_GPIO
#define CY_ADC_POT_PCLK11                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN11                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO12                        PASS0_SAR1
#define CY_ADC_POT_IN_NO12                   CY_ADC_PIN_ADDRESS_AN10
#define CY_ADC_POT_PORT12                         GPIO_PRT12
#define CY_ADC_POT_PIN12                          6
#define CY_ADC_POT_PIN_MUX12                      P12_6_GPIO
#define CY_ADC_POT_PCLK12                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN12                         pass_0_interrupts_sar_32_IRQn

#define CY_ADC_POT_MACRO13                        PASS0_SAR1
#define CY_ADC_POT_IN_NO13                   CY_ADC_PIN_ADDRESS_AN11
#define CY_ADC_POT_PORT13                         GPIO_PRT12
#define CY_ADC_POT_PIN13                          7
#define CY_ADC_POT_PIN_MUX13                      P12_7_GPIO
#define CY_ADC_POT_PCLK13                         PCLK_PASS0_CLOCK_SAR1
#define CY_ADC_POT_IRQN13                         pass_0_interrupts_sar_32_IRQn


#define CY_ADC_POT_MACRO14                        PASS0_SAR2
#define CY_ADC_POT_IN_NO14                   CY_ADC_PIN_ADDRESS_AN0
#define CY_ADC_POT_PORT14                         GPIO_PRT16
#define CY_ADC_POT_PIN14                          0
#define CY_ADC_POT_PIN_MUX14                      P16_0_GPIO
#define CY_ADC_POT_PCLK14                         PCLK_PASS0_CLOCK_SAR2
#define CY_ADC_POT_IRQN14                         pass_0_interrupts_sar_64_IRQn

#define CY_ADC_POT_MACRO15                        PASS0_SAR2
#define CY_ADC_POT_IN_NO15                   CY_ADC_PIN_ADDRESS_AN1
#define CY_ADC_POT_PORT15                         GPIO_PRT16
#define CY_ADC_POT_PIN15                          1
#define CY_ADC_POT_PIN_MUX15                      P16_1_GPIO
#define CY_ADC_POT_PCLK15                         PCLK_PASS0_CLOCK_SAR2
#define CY_ADC_POT_IRQN15                         pass_0_interrupts_sar_64_IRQn

#define CY_ADC_POT_MACRO16                        PASS0_SAR2
#define CY_ADC_POT_IN_NO16                   CY_ADC_PIN_ADDRESS_AN2
#define CY_ADC_POT_PORT16                        GPIO_PRT16
#define CY_ADC_POT_PIN16                          2
#define CY_ADC_POT_PIN_MUX16                      P16_2_GPIO
#define CY_ADC_POT_PCLK16                         PCLK_PASS0_CLOCK_SAR2
#define CY_ADC_POT_IRQN16                         pass_0_interrupts_sar_64_IRQn

#define CY_ADC_POT_MACRO17                        PASS0_SAR2
#define CY_ADC_POT_IN_NO17                   CY_ADC_PIN_ADDRESS_AN3
#define CY_ADC_POT_PORT17                        GPIO_PRT16
#define CY_ADC_POT_PIN17                         3
#define CY_ADC_POT_PIN_MUX17                      P16_3_GPIO
#define CY_ADC_POT_PCLK17                         PCLK_PASS0_CLOCK_SAR2
#define CY_ADC_POT_IRQN17                         pass_0_interrupts_sar_64_IRQn
```

调试log格式整理

```c
*********************T24DG26_R1.0_V1.0.0.3.A888*********************
******************* T24DG26 R1.0_V1.0.0.3.A_ORIN *******************
**************** T24DG26 R1.0_V1.0.0.3.aaaaaaaaaaaaa ***************
******************* T24DG26 R1.0_V1.0.0.3.B_ORIN *******************
**************** T24DG26 R1.0_V1.0.0.3.BBBBBBBBBBBBB ***************
**********************UART_Init_With_Interrupt**********************
**********************UART6_Init_With_Interrupt*********************
**********************UART7_Init_With_Interrupt*********************
**********************UART8_Init_With_Interrupt*********************
**********************UART9_Init_With_Interrupt*********************
*****************************ipc_init OK****************************
***************************tztek_gpio_init**************************
***************************tztek_power_on***************************
*******************************tr_init******************************
*****************************iic_initall****************************
***************************ina3221_initall**************************
************************tztek_lm75bdp_initall***********************
**************************tztek_adc_initall*************************
*****************************Timer_Init*****************************
*****************************lwip_Init******************************
********************T24DG26 R1.0_V1.0.0.3.A_ORIN********************
********************T24DG26 R1.0_V1.0.0.3.B_ORIN********************
******************************** OTA *******************************
*********************** processMessage error ***********************
************************** SCB_UART_RECEIVE ************************
************************* SCB8_UART_RECEIVE ************************
************************* SCB9_UART_RECEIVE ************************
*************************** Exit OTA mode **************************

***************** T24DG26-R2.0-MCU-V1.0.0.0.A_ORIN *****************
************** T24DG26-R2.0-MCU-V1.0.0.0.aaaaaaaaaaaaa *************
***************** T24DG26-R2.0-MCU-V1.0.0.0.B_ORIN *****************
************** T24DG26-R2.0-MCU-V1.0.0.0.BBBBBBBBBBBBB *************

********************* T24DG26-R2.0-MCU-V1.0.0.0 ********************

**************************** PWM_Init ******************************
*********************** user_udp_server_init ***********************
```

