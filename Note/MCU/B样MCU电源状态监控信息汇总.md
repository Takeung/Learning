## B样MCU电源状态监控信息汇总

#### 1 湿度（1 * 3字节）

```c
#define SN_STATUS_HUMIDITY 0u
```

> 示例：80 00 00
> 1、首字节具体到bit位：1000 0000
>    高两位表示状态（00正常、10温度异常），后六位表示序号
> 2、后两个字节表示湿度（%），每四个bit表示一个十进制数位
> 3、解释：湿度异常（为0）、序号为0（SN_STATUS_HUMIDITY）、湿度0%

#### 2 温度（1 * 3字节）

```c
#define SN_STATUS_TEMPERATURE 1u
```

> 示例：81 00 00
> 1、首字节具体到bit位：1000 0001
>    高两位表示状态（00正常、10温度异常），后六位表示序号
> 2、后两个字节表示温度（℃），每四个bit表示一个十进制数位
> 3、解释：温度异常（为0）、序号为1（SN_STATUS_TEMPERATURE）、温度0℃

#### 3 lm75bdp 温度传感器（3 * 3字节）

```c
#define SN_STATUS_LM75BDP_TEMP_1 2u
#define SN_STATUS_LM75BDP_TEMP_2 3u
#define SN_STATUS_LM75BDP_TEMP_3 4u
```

> 示例：02 35 88
> 1、首字节具体到bit位：0000 0002
>    高两位表示状态（00正常、10温度异常），后六位表示序号
> 2、后两个字节表示温度（℃），每四个bit表示一个十进制数位
> 3、解释：温度正常、序号为2（SN_STATUS_LM75BDP_TEMP_1）、温度35.88℃

#### 4 INA3221（5 * 5字节）

```c
#define SN_STATUS_ORIN_12V 5u
#define SN_STATUS_ORIN_5V 6u
#define SN_STATUS_ETH_5V 7u
#define SN_STATUS_CAM_12V 8u
#define SN_STATUS_MM_12V 9u
```

> 示例：85 12 08 00 41
> 1、首字节具体到bit位：1000 0101
>    高两位表示状态（00正常、01电流异常、10电压异常、11压流均异常），后六位表示序号
> 2、后四个字节中，前两个字节表示电压（V）、后两个字节表示电流（A），每四个bit表示一个十进制数位
> 3、解释：电压异常、序号为5（对应 SN_STATUS_ORIN_12V ）、电压12.08V、电流0.41A

#### 5 ADC（13 * 3字节）

```c
#define SN_STATUS_SOC1_VDD_3V3_ADC 10u
#define SN_STATUS_SOC1_VDD_1V8_ADC 11u
#define SN_STATUS_SOC1_VDD_1V2_ADC 12u
#define SN_STATUS_ETH_VDD_3V3_ADC 13u
#define SN_STATUS_ETH_VDD_1V8_ADC 14u
#define SN_STATUS_ETH_VDD_1V5_ADC 15u
#define SN_STATUS_ETH_VDD_1V1_ADC 16u
#define SN_STATUS_ETH_VDD_0V88_ADC 17u
#define SN_STATUS_ETH_VDD_2V5_ADC 18u
#define SN_STATUS_5G_VDD_3V8_ADC 19u
#define SN_STATUS_N20527916 20u
#define SN_STATUS_N20527927 21u
#define SN_STATUS_N20527938 22u
```

> 示例：0a 03 27
> 1、首字节具体到bit位：0000 1010
>    高两位表示状态（00正常、10电压异常），后六位表示序号
> 2、后两个字节表示电压（V），每四个bit表示一个十进制数位
> 3、解释：电压正常、序号为10（SN_STATUS_SOC1_VDD_3V3_ADC）、电压3.27V