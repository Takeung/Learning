# UART指令预识别之重排策略验证记录

测试环境：UART_ReceiveData调度周期为1秒。

测试内容：
1、毫米波雷达下电复位指令，多次；
2、摄像头下电复位指令，多次；
3、切换OTA升级模式，一次。

```bash
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^^^01070500000003
Rearranged command is: ^^^^01070500000003
Checksum mismatch. Expected: 0x00, Calculated: 0xEC
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
10s ---------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************ipc_initall*************************
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*******T24DG26_R1.0_V1.0.0.3.A888**********
tr1 init ok
tr2 init ok
tr3 init ok
tr4 init ok
tr5 init ok
tr_init OK
MCU5G1_PORT_CALLBACK
*********************T24DG26 R1.0_V1.0.0.3.aaaaaaaaaaaaa*************************
iic_initall OK
*********************ina3221_initall*************************
Timer-------- timer clock frequency = 2M
Timer_Init OK
MII_PHYSID1:141
MII_PHYSID2:DD1
lwip_Init OK
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
*******************************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*******T24DG26_R1.0_V1.0.0.3.A888**********
tr1 init ok
tr2 init ok
tr3 init ok
tr4 init ok
tr5 init ok
tr_init OK
MCU5G1_PORT_CALLBACK
*********************T24DG26 R1.0_V1.0.0.3.aaaaaaaaaaaaa*************************
iic_initall OK
*********************ina3221_initall*************************
Timer-------- timer clock frequency = 2M
Timer_Init OK
MII_PHYSID1:141
MII_PHYSID2:DD1
lwip_Init OK
3s -------->>>>>
*******************************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*******T24DG26_R1.0_V1.0.0.3.A888**********
tr1 init ok
tr2 init ok
tr3 init ok
tr4 init ok
tr5 init ok
tr_init OK
MCU5G1_PORT_CALLBACK
*********************T24DG26 R1.0_V1.0.0.3.aaaaaaaaaaaaa*************************
iic_initall OK
*********************ina3221_initall*************************
Timer-------- timer clock frequency = 2M
Timer_Init OK
MII_PHYSID1:141
MII_PHYSID2:DD1
lwip_Init OK
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^01070500000003$$
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: ^^^^01070500000003
Rearranged command is: ^^^^01070500000003
Checksum mismatch. Expected: 0x00, Calculated: 0xEC
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01070500000003
Rearranged command is: ^^01070500000003$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Camera request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$緟蕫蕫聬聬&揀^^
Rearranged command is: ^^(h@d4($$
Received data is too short to be valid.
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: 01060500000002$$^^
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: 01060500000002$$^^
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: 01060500000002$$^^
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: 01060500000002$$^^
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: 01060500000002$$^^
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: 01^^01060500000002
Rearranged command is: 01^^01060500000002
Checksum mismatch. Expected: 0x00, Calculated: 0xED
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^01060500000002
Rearranged command is: ^^01060500000002$$
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
Millimeter-wave radar request for power down!!!!
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
*********************T24DG26 R1.0_V1.0.0.3.A_ORIN*************************
*********************SCB8_UART_RECEIVE*************************
Input command is: $$^^0109050000000D
Rearranged command is: ^^0109050000000D$$
3s -------->>>>>
3s -------->>>>>
10s ---------->>>>>
OTA on request!
 START CLOSE OTHER CORE 
 CM7_0 and CM7_1 CLOSED 
Internal flash operation
1. Press 1 to receive file and flash programming on MAPB_CM0p
2. Press 2 to receive file and flash programming on MAPB_CM7_0 
3. Press 3 to receive file and flash programming on MAPB_CM7_1
4. Press 4 to SWITCH WORK FLASH
```

