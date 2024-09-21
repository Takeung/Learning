# 一体机B样电源监控方案

ADC：

| 功能                   | 控制方式                                  | 检测方式          | 输出 |
| ---------------------- | ----------------------------------------- | ----------------- | ---- |
| SOC1_VDD_3V3电源       | ORIN模组输出SOC_CARRIER_POWER_ON开启      | P10.4 ADC检测电压 | 0    |
| SOC1_VDD_1V8电源       | ORIN模组输出SOC_CARRIER_POWER_ON开启      | P10.5 ADC检测电压 | 1    |
| SOC1_VDD_1V2电源       | ORIN模组输出SOC_CARRIER_POWER_ON开启      | P10.6 ADC检测电压 | 2    |
| ETH_VDD_3V3电源        | ETH_VDD_5V POWER_OK开启                   | P12.2 ADC检测电压 | 3    |
| ETH_VDD_1V8电源        | ETH_VDD_3V3 POWER_OK开启                  | P12.3 ADC检测电压 | 4    |
| ETH_VDD_1V5电源        | ETH_VDD_1V8 POWER_OK开启                  | P12.4 ADC检测电压 | 5    |
| ETH_VDD_1V1电源        | ETH_VDD_1V5 POWER_OK开启                  | P12.5 ADC检测电压 | 6    |
| ETH_VDD_0V88电源       | ETH_VDD_5V POWER_OK开启                   | P12.6 ADC检测电压 | 7    |
| ETH_VDD_2V5电源        | ETH_VDD_5V POWER_OK开启                   | P12.7 ADC检测电压 | 8    |
| 5G_VDD_3V8(5G模组电源) | 预留P3.6控制，默认ETH_VDD_5V POWER_OK开启 | P16.0 ADC检测电压 | 9    |
| 加热丝电源1            | P10.1,高控制加热丝开启                    | P8.1 ADC检测电压  | 10   |
| 加热丝电源2            | P10.2,高控制加热丝开启                    | P8.2 ADC检测电压  | 11   |
| 加热丝电源3            | P10.3,高控制加热丝开启                    | P8.3 ADC检测电压  | 12   |

INA3221：

![image-20240905115704719](/home/tyjt/.config/Typora/typora-user-images/image-20240905115704719.png)

| **功能**                         | **控制方式**               | **检测方式**                             | 输出 |
| -------------------------------- | -------------------------- | ---------------------------------------- | ---- |
| ORIN_HV电源                      | P1.4,高控制ORIN_HV电源开启 | P18.1&P18.2 IIC通过INA3221检测，channel1 | 0    |
| ORIN_MV电源                      | P1.3,高控制ORIN_MV电源开启 | P18.1&P18.2 IIC通过INA3221检测，channel2 | 1    |
| ETH_VDD_5V电源(网络入口电源)     | P2.4,高控制电源开启        | P18.1&P18.2 IIC通过INA3221检测，channel3 | 2    |
| /                                | /                          | /                                        | /    |
| /                                | /                          | /                                        | /    |
| /                                | /                          | /                                        | /    |
| /                                | /                          | /                                        | /    |
| CAM_12V电源(相机POC，加热丝)     | P2.6,高控制电源开启        | P17.2&P17.3 IIC通过INA3221检测，channel2 | 3    |
| MM_12V电源(毫米波、风扇、补光灯) | P2.7,高控制电源开启        | P17.2&P17.3 IIC通过INA3221检测，channel3 | 4    |

