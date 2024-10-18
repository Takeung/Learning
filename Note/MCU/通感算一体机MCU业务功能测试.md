# 通感算一体机MCU业务功能测试

[TOC]

------

## 0 测试环境

在不同的[一体机设备环境](https://tyjt.yuque.com/lz6a2x/lcxodb/xzf7uaiexf9cnotr#8e4d)上，基本业务功能的测试预期可能存在差异（比如A样ORIN同时监测17路ADC、9路INA3221，而B样ORIN仅有13路ADC、5路INA3221，等等），但OTA功能的测试预期各版本之间几乎没有区别。

| 设备    | IP                                                           |
| ------- | ------------------------------------------------------------ |
| A样ORIN | 实验室裸板：172.26.1.131，172.26.1.163<br />2号路口外挂整机：192.168.11.244，192.168.11.245，192.168.11.246，192.168.11.247 |
| B样ORIN | 实验室整机：172.26.1.187，172.26.1.188，172.26.1.206，172.26.1.207<br />无人岛整机：172.26.1.190，172.26.1.191，172.26.1.192，172.26.1.193<br />园区内外挂整机：192.168.11.248，192.168.11.249 |
| A样NX   | 实验室拆盖整机：172.26.1.204（调试中）                       |
| B样NX   | 实验室整机：172.26.1.208（调试中）、172.26.1.212（调试中）   |

实验室整机可以通过公司内网SSH登录访问（账户和密码都是`nvidia`），外挂设备需要挂载VPN或者通过`jumpserver`远程访问（但一般不建议在外挂设备上做功能测试，避免测试失败影响设备正常运行）。

本测试文档中所有测试用例的基本测试流程：

1. 远程登录一体机设备；
2. 通过上位机程序对MCU下发控制指令；
   - MCU控制和升级相关的上位机程序部署在一体机设备的`/Downloads/BIN_*/`路径下，`BIN_*`的格式一般为`BIN_日期_版本号`，截止10月16号，以B样ORIN上最新版本为例，对应路径为`BIN_1008_1_0_0_2`。
3. 观察一体机状态和响应。
   - 状态：供电、系统、温度指示灯，以及闪光灯；
   - 响应：通过串口助手（`Windows：SSCOM`；`Linux：minicom`）查看MCU调试串口的log。

| 设备    | 升级串口     | 调试串口                        |
| ------- | ------------ | ------------------------------- |
| A样ORIN | /dev/ttysWK3 | 无（只能有线连接MCU Debug串口） |
| B样ORIN | /dev/ttysWK3 | /dev/ttysWK0                    |
| A样NX   | /dev/ttysWK0 | 无（只能有线连接MCU Debug串口） |
| B样NX   | /dev/ttysWK0 | /dev/ttyi2csWK1                 |



------

## 1 基本业务功能

MCU的基本业务功能部分即是业务核所负责的各个功能模块，包括各个模块的下电复位功能、MCU的状态监测并向SOC上报的功能，以及MCU心跳监测上报和以太链路（UDP）的功能。

### 1.1 模块下电复位

MCU作为总的供电控制核心，一体机上各个主要模块的下电复位都由它自动或手动完成，自动复位依赖于SOC侧的话题服务控制，在一体机运行异常时由ROS程序发起复位指令通知MCU重启对应单元。

本测试文档只关注MCU侧的基本功能是否稳定可用，故而采用手动复位的方式进行各项测试。

测试人员可以通过串口助手或者自行编写测试脚本，用于对MCU发送控制指令，但建议直接使用已开发的上位机程序。其中，用于控制模块下电复位的上位机程序主要为`/Downloads/BIN_*/mcu-signal-cmd.py`，控制5G模组下电复位的上位机程序为`/Downloads/BIN_*/soc_reset_5g.py`。

#### 1.1.1 MCU整机下电复位

（1）用例1

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 0
```

**预期状态**：指令输入完成后最多10秒内，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：指令输入完成后最多10秒内，调试串口输出log显示设备开始下电重启

```bash
Whole system power-down and reboot request!!!!
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。

#### 1.1.2 ORIN下电复位（仅ORIN）

（1）用例1

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 1
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：指令输入完成后最多10秒内，调试串口输出log显示ORIN开始下电重启

```bash
Orin power-down request!!!!
```

之后程序停止运行，命令行不再响应，大概30~40秒之后，ORIN重启完成，可以重新登录。

#### 1.1.3 NX下电复位（仅NX）

（1）用例1

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 2
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：指令输入完成后最多10秒内，调试串口输出log显示NX开始下电重启

```bash
NX power-down request!!!!
```

之后程序停止运行，命令行不再响应，大概30~40秒之后，NX重启完成，可以重新登录。

#### 1.1.4 开补光灯（仅B样）

（1）用例1

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 3
```

**预期状态**：指令输入完成后最多5秒内，补光灯开启并常亮。

**预期响应**：指令输入完成后最多5秒内，调试串口输出log显示开启补光灯

```bash
Fill light power-on request!!!!
```

之后程序停止运行，命令行可以正常响应。

#### 1.1.5 关补光灯（仅B样）

（1）用例1

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 4
```

**预期状态**：指令输入完成后最多5秒内，补光灯关闭。

**预期响应**：指令输入完成后最多5秒内，调试串口输出log显示关闭补光灯

```bash
Fill light power-down request!!!!
```

之后程序停止运行，命令行可以正常响应。

#### 1.1.6 毫米波雷达下电复位

由于毫米波雷达未设计供电指示灯，故而只能通过`能否ping通网络`的方式确定设备是否下电成功。测试人员可以持续ping目标雷达，同时执行以下用例，观察雷达断链时间，以确定测试结果。

> 注：每台一体机整机设备上的雷达IP可查看：[测试用设备安装信息](https://tyjt.yuque.com/lz6a2x/lcxodb/xzf7uaiexf9cnotr#8e4d)

（1）用例1

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 5
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：指令输入完成后最多10秒内，调试串口输出log显示毫米波雷达开始下电重启

```bash
Millimeter-wave radar power-down and reboot request!!!!
```

之后程序停止运行，命令行可以正常响应。

在指令输入完成后最多10秒内，雷达开始断开，大约16秒后能够重新ping通。

#### 1.1.7 摄像头下电复位

由于摄像头未设计供电指示灯，也没有网络连接，故而无法观测到摄像头的下电现象。

（1）用例1

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 6
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：指令输入完成后最多5秒内，调试串口输出log显示摄像头开始下电重启

```bash
Camera power-down and reboot request!!!!
```

之后程序停止运行，命令行可以正常响应。

#### 1.1.8 5G模组复位（仅ZM9200）

由于5G模组不能直接断电复位，而需通过SOC写复位引脚的方式实现控制，所以在MCU侧并不会看到状态和响应上的变化。测试人员可以持续ping目标5G模组的IP，同时执行以下用例，观察模组断链时间，以确定测试结果。

```bash
ping www.baidu.com -I usb2
```

> 注：5G模组型号为ZM9200的一体机设备中，5G模组连接的网口均为usb2。

（1）用例1

```bash
sudo python3 soc_reset_5g.py
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：调试串口无明显响应。

在指令输入完成后最多2.5秒内，5G模组开始断开，此时尝试ping公网将会收到

```bash
ping: SO_BINDTODEVICE usb2: No such device
```

大概10秒之后，设备恢复识别但不能ping通。在5G模组断开之后将近3分钟时，5G通信恢复。

### 1.2 MCU状态监测

MCU以10秒为周期，持续向SOC上报状态监测数据，数据包以`??`开始，以`!!`结尾，内容包括启动后运行时间计数（单位：10秒）、当前版本及分区号、温度、湿度、LM75BDP温度、INA3221电压电流、ADC电压，各类数据之间以换行符隔开。

测试人员可以借助串口助手等工具对接收到的数据包解析验证，或者自行开发测试脚本验证，以下仅说明解析方法。

- A样状态监测包示例

![](https://s2.loli.net/2024/10/16/n4YrVDiMvC5Qfph.png)

- B样状态监测包示例

![](https://s2.loli.net/2024/10/16/wYKEGm4OzjMdcTF.png)

#### 1.2.1 启动后运行时间计数（单位：10秒)

![](https://s2.loli.net/2024/10/16/h5Igy7zM2WEHpcr.png)

（1）用例1

10秒内不执行任何指令，检查运行时间计数是否连续。

**预期状态**：一体机设备无明显状态变化。

**预期响应**：计数连续，差值为1。

（2）用例2

10秒内执行一次摄像头下电复位指令

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 6
```

检查运行时间计数是否连续。

**预期状态**：一体机设备无明显状态变化。

**预期响应**：计数不连续，差值为2。

（3）用例3

10秒内执行任意次数摄像头下电复位指令

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 6
```

检查运行时间计数是否连续。

**预期状态**：一体机设备无明显状态变化。

**预期响应**：计数不连续，差值为2。

#### 1.2.2 当前版本及分区号

由设备版本号、MCU固件版本号以及运行分区号组成，命名格式如下：

```bash
T24DG26 R + 设备版本号 + _V + MCU固件版本号 + . + 运行分区号
```

其中，设备版本号分为A样（`1.0`）和B样（`2.0`），MCU固件版本号使用四位十进制数表示，四位数字之间用`.`连接，运行分区号只可为`A_ORIN`、`B_ORIN`、`A_NX`以及`B_NX`四者之一。

该部分不设测试用例，测试人员可以根据设备版本匹配校验。

#### 1.2.3 温度

当前实现中，A、B样状态监测数据包中温度数据所处的相对位置并不相同。

（1）A样

![](https://s2.loli.net/2024/10/17/ZTuvPWLhRbryYJs.png)

如图所示，A样设备中温度在监测数据包中序号为`00`，数值为`2529`，对应解析为：测得系统温度`25.29℃`。

（2）B样

![](https://s2.loli.net/2024/10/17/Ud6i83bAQY5cLq2.png)

如图所示，B样设备中温度在监测数据包中序号为`01`，数值为`4088`，对应解析为：测得系统温度`40.88℃`。

#### 1.2.4 湿度

当前实现中，A、B样状态监测数据包中温度数据所处的相对位置并不相同。

（1）A样

![](https://s2.loli.net/2024/10/17/dJDWbfNROQiBumr.png)

如图所示，A样设备中湿度在监测数据包中序号为`01`，数值为`4815`，对应解析为：测得系统湿度`48.15%`。

（2）B样

![](https://s2.loli.net/2024/10/17/Iw6Y9HvZiTElOct.png)

如图所示，B样设备中湿度在监测数据包中序号为`00`，数值为`3752`，对应解析为：测得系统湿度`37.52%`。

#### 1.2.5 LM75BDP温度（仅B样）

当前实现中，只有B样添加了三路板上LM75BDP温度传感器的数据上报。

![](https://s2.loli.net/2024/10/17/Qxkv2Jr86PtgE1d.png)

如图所示，三路板上LM75BDP在监控数据包中序号对应为`02`~`03`，数值对应为`4825`、`4838`和`4838`，对应解析为：测得三路板上温度`48.25℃`、`48.38℃`、`48.38℃`。

#### 1.2.6 INA3221电压电流监测

每一路INA3221监测数据由5个字节组成，使用十六进制编码，编码组成如下：

```bash
首字节（端口状态/包中序号） + 第2、3字节（电压） + 第4、5字节（电流）
```

首字节编码格式解析如下：

```
BIT7 BIT6 BIT5 BIT4   BIT3 BIT2 BIT1 BIT0 
```

`BIT0`~`BIT5`共六位用于表示当前数据的包中序号，`BIT6`表征电流异常状态（0:正常/1:异常），`BIT7`表征电压异常状态（0:正常/1:异常）。

当前实现中，A样设计了9路INA3221的电压电流监控，B样只有5路。

（1）A样

| 包中序号 | 监测端口     | 预期电压 |
| -------- | ------------ | -------- |
| 02       | ORIN_12V     | 12V      |
| 03       | ORIN_5V      | 5V       |
| 04       | NX_5V_1      | 5V       |
| 05       | NX_5V_2      | 5V       |
| 06       | ETH_5V       | 5V       |
| 07       | PCIE_5V      | 5V       |
| 08       | LASER_24V    | 24V      |
| 09       | 8838_12V     | 12V      |
| 0a       | MM_RADAR_12V | 12V      |

![](https://s2.loli.net/2024/10/17/KaJV4OdIuv3Ytnh.png)

如图所示，

用例1中，`端口状态/包中序号`为`03`，测得数值为`05000165`，对应解析为：监测端口`ORIN_5V`，测得电压`5.00V`，电流`1.65A`，电压、电流均处于正常区间内。

用例2中，`端口状态/包中序号`为`c5`，测得数值为`01840000`，对应解析为：监测端口`NX_5V_2`，测得电压`1.84V`，电流`0.00A`，电压、电流均异常。

用例3中，`端口状态/包中序号`为`49`，测得数值为`11960000`，对应解析为：监测端口`8838_12V`，测得电压`11.96V`，电流`0.00A`，电压处于正常区间内、电流异常。

（2）B样

| 包中序号 | 监测端口     | 预期电压 |
| -------- | ------------ | -------- |
| 05       | ORIN_12V     | 12V      |
| 06       | ORIN_5V      | 5V       |
| 07       | ETH_5V       | 5V       |
| 08       | CAM_12V      | 12V      |
| 09       | MM_RADAR_12V | 12V      |

![](https://s2.loli.net/2024/10/17/tud5ifVAUBQklaz.png)

如图所示，`端口状态/包中序号`为`06`，测得数值为`05010203`，对应解析为：监测端口`ORIN_5V`，测得电压`5.01V`，电流`2.03A`，电压、电流均处于正常区间内。

#### 1.2.7 ADC电压监测

当前实现中，A样设计了17路ADC电压监控，B样只有13路。

（1）A样

| 包中序号 | 监测端口          | 预期电压 |
| -------- | ----------------- | -------- |
| 0b       | SOC1_VDD_3V3_ADC  | 3.3V     |
| 0c       | SOC1_VDD_1V8_ADC  | 1.8V     |
| 0d       | SOC2_VDD_3V3_ADC  | 3.3V     |
| 0e       | SOC2_VDD_1V8_ADC  | 1.8V     |
| 0f       | ETH_VDD_3V3_ADC   | 3.3V     |
| 10       | ETH_VDD_1V8_ADC   | 1.8V     |
| 11       | ETH_VDD_1V5_ADC   | 1.5V     |
| 12       | ETH_VDD_1V1_ADC   | 1.1V     |
| 13       | ETH_VDD_0V88_ADC  | 0.88V    |
| 14       | ETH_VDD_1V2_ADC   | 1.2V     |
| 15       | 5G_VDD_3V8_ADC    | 3.8V     |
| 16       | PCIE_VDD_1V8_ADC  | 1.8V     |
| 17       | PCIE_VDD_3V3_ADC  | 3.3V     |
| 18       | LED_VDD_3V3_M_ADC | 3.3V     |
| 19       | N20527916         | ≤ 1.65V  |
| 1a       | N20527927         | ≤ 1.65V  |
| 1b       | N20527938         | ≤ 1.65V  |

![](https://s2.loli.net/2024/10/17/ifqWcp7STA69UDl.png)

如图所示，

用例1中，`端口状态/包中序号`为`0c`，测得数值为`0180`，对应解析为：监测端口`SOC1_VDD_1V8_ADC`，测得电压`1.80V`，电压处于正常区间内。

用例2中，`端口状态/包中序号`为`8e`，测得数值为`0005`，对应解析为：监测端口`SOC2_VDD_1V8_ADC`，测得电压`0.05V`，电压异常。

（2）B样

| 包中序号 | 监测端口         | 预期电压 |
| -------- | ---------------- | -------- |
| 0a       | SOC1_VDD_3V3_ADC | 3.3V     |
| 0b       | SOC1_VDD_1V8_ADC | 1.8V     |
| 0c       | SOC1_VDD_1V2_ADC | 1.2V     |
| 0d       | ETH_VDD_3V3_ADC  | 3.3V     |
| 0e       | ETH_VDD_1V8_ADC  | 1.8V     |
| 0f       | ETH_VDD_1V5_ADC  | 1.5V     |
| 10       | ETH_VDD_1V1_ADC  | 1.1V     |
| 11       | ETH_VDD_0V88_ADC | 0.88V    |
| 12       | ETH_VDD_2V5_ADC  | 2.5V     |
| 13       | 5G_VDD_3V8_ADC   | 3.8V     |
| 14       | N20527916        | ≤ 1.65V  |
| 15       | N20527927        | ≤ 1.65V  |
| 16       | N20527938        | ≤ 1.65V  |

![](https://s2.loli.net/2024/10/17/B9DoyXHNaATprhI.png)

如图所示，`端口状态/包中序号`为`0b`，测得数值为`0182`，对应解析为：监测端口`SOC1_VDD_1V8_ADC`，测得电压`1.82V`，电压处于正常区间内。

### 1.3 心跳监测（仅B样）

MCU始终对SOC发送心跳信号，占空比2:1，频率1Hz；MCU检测到INA3221电压电流或者ADC电压异常时，向SOC发送告警信号，占空比1:1，根据严重程度频率分别为5或10Hz。

测试人员可以自行编写测试脚本读取SOC设备上`/dev/mcu-gpio4`的电平信号，绘制出波形并计算频率和占空比但建议直接使用已开发的上位机程序`/Downloads/BIN_*/mcu-signal-cmd.py`，监测命令如下：

```bash
sudo python3 mcu-signal-cmd.py monitor -g /dev/mcu-gpio4
```

**预期状态**：一体机设备无明显变化。

**预期响应**：命令行持续打印输出当前检测到GPIO电平信号的频率和占空比，打印频率与心跳频率一致，格式如下：

```bash
Frequency: 频率, Duty Cycle: 占空比
```

（1）正例

没有电压、电流异常时，SOC接收到的心跳信号

![](https://s2.loli.net/2024/10/17/6gS2ozMA7CRlEUN.png)

（2）反例1

当电压异常波动幅值超过标准值5%以上、但未达到10%，且未出现电流异常时，SOC接收到的心跳信号

```bash
Frequency: 5.00 Hz, Duty Cycle: 50.00%
```

同时，调试串口会输出log显示具体是哪个通道的电压或者电流波动超出正常阈值

- ```bash
  The 包中序号 INA3221 channel v_diff_ratio 偏差比例 > 5% while v_measured: 测得电压 and v_nominal: 标准电压
  ```

- ```bash
  The 包中序号 ADC channel v_diff_ratio 偏差比例 > 5% while v_measured: 测得电压 and v_nominal: 标准电压
  ```

（3）反例2

当电压异常波动幅值超过标准值10%以上，或者出现电流异常时，SOC接收到的心跳信号

```bash
Frequency: 10.00 Hz, Duty Cycle: 50.00%
```

同时，调试串口会输出log显示具体是哪个通道的电压或者电流波动超出正常阈值

- ```bash
  The 包中序号 INA3221 channel v_diff_ratio 偏差比例 > 10% while v_measured: 测得电压 and v_nominal: 标准电压
  ```

- ```bash
  The 包中序号 INA3221 channel i_measured 测得电流 < 1e-6
  ```

- ```bash
  The 包中序号 ADC channel v_diff_ratio 偏差比例 > 10% while v_measured: 测得电压 and v_nominal: 标准电压
  ```

### 1.4 以太链路（仅B样）

以太链路指的是MCU与SOC间通过以太网实现的有线通信，与UART串口通信互为冗余，理论上，前述所有可以通过UART串口发送的控制指令，都可以转为通过以太链路发送并达成同样的控制效果，通信协议为UDP。

此外，当设备重启后，SOC首次向MCU发送任意UDP报文后，MCU会以10秒为周期，持续向SOC发送状态监测数据包，直到下一次设备重启后停止。

（1）用例1：MCU未收到上位机指令不上送状态监测数据包

一体机重启后，登录到SOC的远程桌面后，打开`WireShark`程序开始抓包，监测从MCU（IP为`192.168.11.110`、Port为`13137`）上收到的UDP数据包。

**预期状态**：一体机无明显状态变化。

**预期响应**：SOC并不能收到从MCU发来的UDP数据包。

（2）用例2：MCU整机下电复位

```bash
sudo python3 mcu-signal-cmd.py send -m udp -s 0
```

**预期状态**：指令输入完成后最多10秒内，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：指令输入完成后最多10秒内，调试串口输出log显示设备开始下电重启

```bash
Whole system power-down and reboot request!!!!
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。

（3）用例3：ORIN下电复位（仅ORIN设备）

```bash
sudo python3 mcu-signal-cmd.py send -m udp -s 1
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：指令输入完成后最多10秒内，调试串口输出log显示ORIN开始下电重启

```bash
Orin power-down request!!!!
```

之后程序停止运行，命令行不再响应，大概30~40秒之后，ORIN重启完成，可以重新登录。

（4）用例4：NX下电复位（仅NX设备）

```bash
sudo python3 mcu-signal-cmd.py send -m udp -s 2
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：指令输入完成后最多10秒内，调试串口输出log显示NX开始下电重启

```bash
NX power-down request!!!!
```

之后程序停止运行，命令行不再响应，大概30~40秒之后，NX重启完成，可以重新登录。

（5）用例5：开补光灯（仅B样设备）

```bash
sudo python3 mcu-signal-cmd.py send -m udp -s 3
```

**预期状态**：指令输入完成后最多5秒内，补光灯开启并常亮。

**预期响应**：指令输入完成后最多5秒内，调试串口输出log显示开启补光灯

```bash
Fill light power-on request!!!!
```

之后程序停止运行，命令行可以正常响应。

（6）用例6：关补光灯（仅B样设备）

```bash
sudo python3 mcu-signal-cmd.py send -m udp -s 4
```

**预期状态**：指令输入完成后最多5秒内，补光灯关闭。

**预期响应**：指令输入完成后最多5秒内，调试串口输出log显示关闭补光灯

```bash
Fill light power-down request!!!!
```

之后程序停止运行，命令行可以正常响应。

（7）用例7：毫米波雷达下电复位

```bash
sudo python3 mcu-signal-cmd.py send -m udp -s 5
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：指令输入完成后最多10秒内，调试串口输出log显示毫米波雷达开始下电重启

```bash
Millimeter-wave radar power-down and reboot request!!!!
```

之后程序停止运行，命令行可以正常响应。

在指令输入完成后最多10秒内，雷达开始断开，大约16秒后能够重新ping通。

（8）用例8：摄像头下电复位

```bash
sudo python3 mcu-signal-cmd.py send -m udp -s 6
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：指令输入完成后最多5秒内，调试串口输出log显示摄像头开始下电重启

```bash
Camera power-down and reboot request!!!!
```

之后程序停止运行，命令行可以正常响应。

（9）用例9：MCU收到上位机指令后周期上送状态监测数据包

一体机重启后，执行上述任意一条控制指令后，登录到SOC的远程桌面，打开`WireShark`程序开始抓包，监测从MCU（IP为`192.168.11.110`、Port为`13137`）上收到的UDP数据包。

**预期状态**：一体机无明显状态变化。

**预期响应**：SOC能接收到从MCU发来的UDP数据包，周期为10秒。

![](https://s2.loli.net/2024/10/17/ziLFOmnXtpJZBMs.png)



------

## 2 OTA升级

MCU作为OTA升级的从端设备，其主要功能实现都由主核完成，在主端SOC上运行OTA升级上位机程序、通过MCU与SOC间的UART串口传输升级包，再由MCU主核接收并刷写对应分区，进而实现MCU的固件版本升级。

测试人员可以直接使用已经编译好的可执行文件`async_io_uploader.out`，或者对`async_io_uploader.c`自行编译得到可执行文件

```bash
gcc -o async_io_uploader.out async_io_uploader.c
```

以下所有用例都是基于`async_io_uploader.out`完成，以下简称上位机。

### 2.1 多种升级模式

针对不同场景下的不同设备，采取了多样的升级策略。

| 升级模式               | 模式说明                                                     | 升级场景                                                     |
| ---------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 只收模式               | 可以作为串口助手使用                                         | 适用于升级之前运行该模式获取当前运行分区，从而确认待刷写的目标分区 |
| 交互模式               | 将升级所需的各个子项做了拆分，包括：`进入OTA`、`刷CM0`、`刷CM7_0`、`刷CM7_1`、`切换分区`、`退出OTA` | 对照调试log交互执行各个升级子项，一旦某个子项运行出错，可以及时重试 |
| 一键模式（所有核）     | 一键连续执行`进入OTA-刷CM0-刷CM7_0-刷CM7_1-切换分区`         | 适用于主核代码存在修改的情况下，依赖于OTA升级下位机程序版本的稳定，可以减少人为介入，提升升级效率 |
| 一键模式（只刷业务核） | 一键连续执行`进入OTA-刷CM7_0-刷CM7_1-切换分区`               | 适用于主核代码未做修改的情况下，依赖于OTA升级下位机程序版本的稳定，可以减少人为介入，提升升级效率 |

#### 2.1.1 只收模式

（1）用例1

```bash
sudo ./async_io_uploader.out
```

**预期状态**：一体机设备无明显状态变化。

**预期响应**：升级串口输出log，以10秒为周期，持续显示接收到的状态监测数据。

#### 2.1.2 交互模式

首先通过前述只收模式获取当前运行分区，以确认待刷写的目标分区，并执行命令

```bash
sudo ./async_io_uploader.out 目标分区
```

例如当前运行分区为`B_ORIN`，则目标分区为`A_ORIN`，反之亦反；NX设备同理。目标分区只能为`A_ORIN`、`B_ORIN`、`A_NX`、`B_NX`四者之一。

以下用例中以目标分区为A_ORIN用于说明，在正常升级流程中，以下四条用例必须严格按照顺序在180秒内连续完成，否则即视为升级失败。

（1）用例1

输入：0

**预期状态**：一体机设备无明显状态变化。

**预期响应**：升级串口输出log显示进入OTA模式，并重新打印升级菜单。

```bash
Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
0
 Input CMD is: 0
 
 Send CMD for OTA OK!


Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
```

调试串口以1秒为周期，输出log显示进入OTA模式，状态监测数据包停发。

```bash
******************************** OTA *******************************
 START CLOSE OTHER CORE 
 CM7_0 and CM7_1 CLOSED 
g_ota_timer_cnt is: 1

Internal flash operation
1. Press 1 to receive file and flash programming on MAPA_CM0p
2. Press 2 to receive file and flash programming on MAPA_CM7_0 
3. Press 3 to receive file and flash programming on MAPA_CM7_1
4. Press 4 to SWITCH WORK FLASH
```

（2）用例2

输入：1

**预期状态**：一体机设备无明显状态变化。

**预期响应**：升级串口输出log显示CM0核开始刷写以及刷写成功。

```bash
Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
1
 Input CMD is: 1
 
 Send CM0 App Start
 Send CM0 App and Flash OK


Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
```

调试串口以1秒为周期，输出log显示升级过程以及结果。

```bash
Received u8ReadBuf is: 1
g_ota_timer_cnt is: 10
g_ota_timer_cnt is: 11
g_ota_timer_cnt is: 12
g_ota_timer_cnt is: 13
g_ota_timer_cnt is: 14
g_ota_timer_cnt is: 15

Start receive and program (max. 512KB) 
Added 10 seconds delay for user to prepare the Xmodem sender 
g_ota_timer_cnt is: 16
g_ota_timer_cnt is: 17
g_ota_timer_cnt is: 18
g_ota_timer_cnt is: 19
g_ota_timer_cnt is: 20
g_ota_timer_cnt is: 21
g_ota_timer_cnt is: 22
g_ota_timer_cnt is: 23
g_ota_timer_cnt is: 24
g_ota_timer_cnt is: 25
Main_MenuInternalFlash_receive: Receive Start!
g_ota_timer_cnt is: 26
g_ota_timer_cnt is: 27
g_ota_timer_cnt is: 28
g_ota_timer_cnt is: 29
Xmodem: Total size received: 30848 bytes.
Main_MenuInternalFlash_receive: Receive Done!

 Completed Success 

Internal flash operation
1. Press 1 to receive file and flash programming on MAPA_CM0p
2. Press 2 to receive file and flash programming on MAPA_CM7_0 
3. Press 3 to receive file and flash programming on MAPA_CM7_1
4. Press 4 to SWITCH WORK FLASH
```

（3）用例3

输入：2

**预期状态**：一体机设备无明显状态变化。

**预期响应**：升级串口输出log显示CM7_0核开始刷写以及刷写成功。

```bash
Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
2
 Input CMD is: 2
 
 Send CM7_0 App Start
 Send CM7_0 App and Flash OK


Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
```

调试串口以1秒为周期，输出log显示升级过程以及结果。

```bash
Received u8ReadBuf is: 2
g_ota_timer_cnt is: 37
g_ota_timer_cnt is: 38
g_ota_timer_cnt is: 39
g_ota_timer_cnt is: 40
g_ota_timer_cnt is: 41

Start receive and program (max. 512KB) 
Added 10 seconds delay for user to prepare the Xmodem sender 
g_ota_timer_cnt is: 42
g_ota_timer_cnt is: 43
g_ota_timer_cnt is: 44
g_ota_timer_cnt is: 45
g_ota_timer_cnt is: 46
g_ota_timer_cnt is: 47
g_ota_timer_cnt is: 48
g_ota_timer_cnt is: 49
g_ota_timer_cnt is: 50
g_ota_timer_cnt is: 51
Main_MenuInternalFlash_receive: Receive Start!
g_ota_timer_cnt is: 52
g_ota_timer_cnt is: 53
g_ota_timer_cnt is: 54
g_ota_timer_cnt is: 55
g_ota_timer_cnt is: 56
g_ota_timer_cnt is: 57
g_ota_timer_cnt is: 58
g_ota_timer_cnt is: 59
g_ota_timer_cnt is: 60
g_ota_timer_cnt is: 61
g_ota_timer_cnt is: 62
g_ota_timer_cnt is: 63
g_ota_timer_cnt is: 64
g_ota_timer_cnt is: 65
g_ota_timer_cnt is: 66
Xmodem: Total size received: 113792 bytes.
Main_MenuInternalFlash_receive: Receive Done!

 Completed Success 

Internal flash operation
1. Press 1 to receive file and flash programming on MAPA_CM0p
2. Press 2 to receive file and flash programming on MAPA_CM7_0 
3. Press 3 to receive file and flash programming on MAPA_CM7_1
4. Press 4 to SWITCH WORK FLASH
```

（4）用例4

输入：3

**预期状态**：一体机设备无明显状态变化。

**预期响应**：升级串口输出log显示CM7_1核开始刷写以及刷写成功。

```bash
Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
3
 Input CMD is: 3
 
 Send CM7_1 App Start
 Send CM7_1 App and Flash OK


Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
```

调试串口以1秒为周期，输出log显示升级过程以及结果。

```bash
Received u8ReadBuf is: 3

Start receive and program (max. 512KB) 
Added 10 seconds delay for user to prepare the Xmodem sender 
g_ota_timer_cnt is: 74
g_ota_timer_cnt is: 75
g_ota_timer_cnt is: 76
g_ota_timer_cnt is: 77
g_ota_timer_cnt is: 78
g_ota_timer_cnt is: 79
g_ota_timer_cnt is: 80
g_ota_timer_cnt is: 81
g_ota_timer_cnt is: 82
g_ota_timer_cnt is: 83
Main_MenuInternalFlash_receive: Receive Start!
Xmodem: Total size received: 6272 bytes.
Main_MenuInternalFlash_receive: Receive Done!

 Completed Success 

Internal flash operation
1. Press 1 to receive file and flash programming on MAPA_CM0p
2. Press 2 to receive file and flash programming on MAPA_CM7_0 
3. Press 3 to receive file and flash programming on MAPA_CM7_1
4. Press 4 to SWITCH WORK FLASH
```

（5）用例5

输入：4

**预期状态**：补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示刷写完成开始切换分区。

```bash
Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
4
 Input CMD is: 4

 Switch Workflash A Start. Waiting for Reboot!
```

调试串口以1秒为周期，输出log显示刷写完成开始切换分区。

```bash
Received u8ReadBuf is: 4
g_ota_timer_cnt is: 91

SWITCH WORK FLASH A!!
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。重新登录之后查看接收到的状态监测数据包，解析出当前运行分区的信息，即可确认运行分区已从B区切换为A区。

#### 2.1.3 一键模式（所有核）

（1）用例1

确认待刷写的目标分区后，执行命令

```bash
sudo ./async_io_uploader.out 目标分区 all
```

在交互模式的指令基础上后接关键字`all`，即可一键连续执行指令`进入OTA-刷CM0-刷CM7_0-刷CM7_1-切换分区`。

**预期状态**：开始刷写后一体机状态无明显变化，待`切换分区`指令输入完成后最多3秒内，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口逐次输出log显示升级子项的运行状态和结果，所有动作正常完成后，执行切换分区，详细log可以参考前述交互模式下的介绍；如果其中某个子项升级失败，则会返回打印

```bash
 Send CM0/CM7_0/CM7_1 App Failed
```

此时程序会退出一键模式，用户可以切换到交互模式，继续执行失败的指令。

调试串口以1秒为周期，逐次输出log显示升级过程，所有动作正常完成后，执行切换分区，详细log可以参考前述交互模式下的介绍；如果其中某个子项升级失败，则会打印下述log的其中之一：

```bash
 Remote Cancel
 Sync Error 
 Too Many Retry 
 Flash Error 
```

之后会持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

 此时程序会退出一键模式，用户可以切换到交互模式，继续执行失败的指令，但所有动作必须在开始进入该模式后的180秒内全部完成。

#### 2.1.4 一键模式（只刷业务核）

（1）用例1

确认待刷写的目标分区后，执行命令

```bash
sudo ./async_io_uploader.out 目标分区 true
```

在交互模式的指令基础上后接关键字`true`，即可一键连续执行指令`进入OTA-刷CM7_0-刷CM7_1-切换分区`。

**预期状态**：开始刷写后一体机状态无明显变化，待`切换分区`指令输入完成后最多3秒内，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口逐次输出log显示升级子项的状态和结果，所有动作正常完成后，执行切换分区，详细log可以参考前述交互模式下的介绍；如果其中某个子项升级失败，则会返回打印

```bash
 Send CM0/CM7_0/CM7_1 App Failed
```

此时程序会退出一键模式，用户可以切换到交互模式，继续执行失败的指令。

调试串口以1秒为周期，逐次输出log显示升级过程，所有动作正常完成后，执行切换分区，详细log可以参考前述交互模式下的介绍；如果其中某个子项升级失败，则会打印下述log的其中之一：

```bash
 Remote Cancel
 Sync Error 
 Too Many Retry 
 Flash Error 
```

之后会持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

 此时程序会退出一键模式，用户可以切换到交互模式，继续执行失败的指令，但所有动作必须在开始进入该模式后的180秒内全部完成。

### 2.2 超时退出机制

由于OTA模式下，业务核被关闭使能、基本业务功能失效，为了及时恢复一体机业务、避免主核长时间运行在OTA模式或者传输挂死的情况，所以设计实现了OTA超时退出机制。

该机制主要包含两个子项：60秒未完成单核刷写即视为超时，触发定时器中断强制退出OTA模式；进入OTA模式后180秒无论是否完成升级都会被视为超时，触发定时器中断强制重启恢复一体机业务运行。

#### 2.2.1 启动单次刷写60秒超时退出

该机制无法手动触发反例，正常单次刷写始终能保持在30秒之内完成，可以持续多次反复测试，只有发生传输失败导致串口阻塞时才会触发该机制，致使超时退出。

（1）正例

进入OTA升级交互模式后，执行指令`进入OTA`，之后顺序执行`刷CM0-刷CM7_0-刷CM7_1`指令，完成后继续执行`切换分区`,，所有指令务必在180秒内完成。

**预期状态**：从指令`进入OTA`输入完成后开始，到指令`切换分区`输入完成之前，一体机无明显状态变化；完成所有输入之后，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示顺利进入OTA模式后，逐次显示升级过程和结果，每次升级子项完成都会回到升级菜单界面。

调试串口以1秒为周期，持续输出log，在显示顺利进入OTA模式后逐次显示升级过程和结果，并持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

刷写完成、执行`切换分区`指令后，输出log显示刷写完成、开始切换分区。

```
Received u8ReadBuf is: 4
g_ota_timer_cnt is: 时间计数

SWITCH WORK FLASH A!!
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。

#### 2.2.2 进入OTA模式后180秒超时重启

（1）用例1

执行指令

```bash
sudo python3 mcu-signal-cmd.py send -m uart -s 7
```

或者，进入OTA升级交互模式后，执行指令`进入OTA`

**预期状态**：从指令`进入OTA`输入完成后开始计时，180秒内无明显状态变化；180秒后，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示顺利进入OTA模式，并停留在升级菜单界面。

```bash
Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
```

调试串口以1秒为周期，持续输出log显示顺利进入OTA模式，并持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

 时间计数最多更新到`180`，尔后log显示

```txt
*********************** OTA timeout restart ************************
```

程序则停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。

（2）用例2

进入OTA升级交互模式后，执行指令`进入OTA`，之后执行`刷CM0/刷CM7_0/刷CM7_1`指令中的任意组合，但不继续执行`切换分区/退出OTA`，所有指令务必在180秒内完成。

**预期状态**：从指令`进入OTA`输入完成后开始计时，180秒内无明显状态变化；180秒后，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示顺利进入OTA模式后，逐次显示升级过程和结果，每次升级子项完成都会回到升级菜单界面。

调试串口以1秒为周期，持续输出log，在显示顺利进入OTA模式后逐次显示升级过程和结果，并持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

 时间计数最多更新到`180`，尔后log显示

```bash
*********************** OTA timeout restart ************************
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。

### 2.3 切换分区并重启

MCU在A、B分区各自存储了一套程序包，两者存在少许差异，但核心业务功能保持一致，互为冗余。`切换分区`主要发生在OTA升级和版本回退的场景下，当每次OTA刷写完目标分区后，需要执行`切换分区`指令切换到目标分区并运行新刷入的程序包；而一旦当前分区的业务代码出现运行挂死的情况，主核会主动执行`切换分区`的指令以切换到另一个分区并运行新分区的程序包，保证一体机业务能够及时恢复运行。

以当前分区为A区为例，执行`切换分区`指令后重新登录，MCU固件运行的分区会相应切换为B区；反之亦反。

（1）用例1

进入OTA升级交互模式后，执行指令`进入OTA`，之后在180秒内执行`切换分区`。

**预期状态**：从指令`切换分区`输入完成约1秒后，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示开始切换分区。

```bash
Enter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?
4
 Input CMD is: 4

 Switch Workflash B Start. Waiting for Reboot!
```

调试串口以1秒为周期，输出log显示刷写完成开始切换分区。

```bash
Received u8ReadBuf is: 4
g_ota_timer_cnt is: 12

SWITCH WORK FLASH B!!
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。重新登录之后查看接收到的状态监测数据包，解析出当前运行分区的信息，即可确认运行分区已从A区切换为B区。

（2）用例2

进入OTA升级交互模式后，执行指令`进入OTA`，之后顺序执行`刷CM0-刷CM7_0-刷CM7_1`指令，完成后继续执行`切换分区`,，所有指令务必在180秒内完成。

**预期状态**：从指令`进入OTA`输入完成后开始，到指令`切换分区`输入完成之前，一体机无明显状态变化；完成所有输入之后，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示顺利进入OTA模式后，逐次显示升级过程和结果，每次升级完成都会回到升级菜单界面。

调试串口以1秒为周期，持续输出log，在显示顺利进入OTA模式后逐次显示升级过程和结果，并持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

刷写完成、执行`切换分区`指令后，输出log显示刷写完成、开始切换分区。

```
Received u8ReadBuf is: 4
g_ota_timer_cnt is: 时间计数

SWITCH WORK FLASH A!!
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。重新登录之后查看接收到的状态监测数据包，解析出当前运行分区的信息，即可确认运行分区已从B区切换为A区。

（3）用例3

进入OTA升级交互模式后，执行指令`进入OTA`，之后顺序执行`刷CM7_0-刷CM7_1`指令，完成后继续执行`切换分区`，所有指令务必在180秒内完成。

**预期状态**：从指令`进入OTA`输入完成后开始，到指令`切换分区`输入完成之前，一体机无明显状态变化；完成所有输入之后，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示顺利进入OTA模式后，逐次显示升级过程和结果，每次升级完成都会回到升级菜单界面。

调试串口以1秒为周期，持续输出log，在显示顺利进入OTA模式后逐次显示升级过程和结果，并持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

刷写完成、执行`切换分区`指令后，输出log显示刷写完成、开始切换分区。

```
Received u8ReadBuf is: 4
g_ota_timer_cnt is: 时间计数

SWITCH WORK FLASH A!!
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。重新登录之后查看接收到的状态监测数据包，解析出当前运行分区的信息，即可确认运行分区已从B区切换为A区。

### 2.4 OTA模式中断退出

进入OTA模式后可以通过执行指令`退出OTA`来主动退出该模式。

（1）用例1

进入OTA升级交互模式后，执行指令`进入OTA`，之后在180秒内执行`退出OTA`指令。

**预期状态**：从指令`进入OTA`输入完成后开始，到指令`退出OTA`输入完成之前，一体机无明显状态变化；完成所有输入之后，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示顺利进入OTA模式后，回到升级菜单界面；在指令`退出OTA`输入完成后，输出log显示

```bash
 Quit from Upper Machine!
```

调试串口以1秒为周期，持续输出log，在显示顺利进入OTA模式后持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

待执行`退出OTA`指令后，输出log显示退出上位机，一体机设备开始重启。

```bash
*************************** Exit OTA mode **************************
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。重新登录之后查看接收到的状态监测数据包，解析出当前运行分区的信息，即可确认运行分区并未改变。

（2）用例2

进入OTA升级交互模式后，执行指令`进入OTA`，之后执行`刷CM0/刷CM7_0/刷CM7_1`指令中的任意组合，最终执行`退出OTA`，所有指令务必在180秒内完成。

**预期状态**：从指令`进入OTA`输入完成后开始，到指令`退出OTA`输入完成之前，一体机无明显状态变化；完成所有输入之后，补光灯短暂亮起后关闭，供电、系统、温度指示灯关闭，MCU开始断电重启，等待几秒后，补光灯重新亮起，1秒后关闭，供电、系统、温度指示灯开启并常亮，大概55~60秒之后，设备重启完成，可以重新登录。

**预期响应**：升级串口输出log显示顺利进入OTA模式后，逐次显示升级过程和结果，每次升级完成都会回到升级菜单界面；在指令`退出OTA`输入完成后，输出log显示

```bash
 Quit from Upper Machine!
```

调试串口以1秒为周期，持续输出log，在显示顺利进入OTA模式后持续打印

```bash
g_ota_timer_cnt is: 时间计数
```

待执行`退出OTA`指令后，输出log显示退出上位机，一体机设备开始重启。

```bash
*************************** Exit OTA mode **************************
```

之后程序停止运行，命令行不再响应，大概55~60秒之后，设备重启完成，可以重新登录。重新登录之后查看接收到的状态监测数据包，解析出当前运行分区的信息，即可确认运行分区并未改变。