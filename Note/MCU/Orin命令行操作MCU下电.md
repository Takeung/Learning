# Orin命令行操作MCU下电

1、连接USB串口后，启动sscom串口助手，选择正确的COM端口后点击打开串口。

![](https://s2.loli.net/2024/07/09/eLiTjVYBIzvxuFm.png)

在接收窗口能够观察到日志，证明正确连接。

2、通过SSH登录Orin

```bash
账号：ssh nvidia@172.26.1.131
密码：nvidia
```

![](https://s2.loli.net/2024/07/09/nxwJRL9oPE6tijV.png)

显示上述图片信息则表明登录成功。

3、启动Orin控制台界面

```bash
sudo minicom -s
```

![](https://s2.loli.net/2024/07/09/syX75RYuQZPbEaL.png)

4、顺利进入后选择Serial port setup，修改串口设备名为下图中所示（不确定是否为必要操作）

![](https://s2.loli.net/2024/07/09/L8c3V7AgRObjkMv.png)

查看minicom可用指令列表

![](https://s2.loli.net/2024/07/09/7yP9TqxeLR3tsjI.png)

5、在minicom菜单界面选择Exit退出，之后输入下述指令下电

```bash
^^01010500000005$$
```

![](https://s2.loli.net/2024/07/09/a7GoXsQAOPe3zjJ.png)

此外，可以通过python脚本发送下电请求

```python
#!/usr/bin/env python3
import subprocess
import os
import sys
import time
import serial
import argparse
import serial

# 定义要发送的字符串列表

strings = [
    "^^01030500000007$$",
    "^^01050500000001$$",
    "^^01020500000006$$",
    "^^01010500000005$$",
    "^^01060500000002$$",
    "^^01070500000003$$",
    "^^0109050000000D$$"
]
print('选择0:给Nx单独下电指令\n选择1:给Nx单独上电指令\n选择2:给ORIN下电指令\n选择3:给MCU下电指令\n选择4:给毫米波雷达下电指令\n选择5:给摄像头下电指令\n选择6:OTA升级'')

# 设置argparse

parser = argparse.ArgumentParser(description='通过串口发送指定字符串')
parser.add_argument('string_index', type=int, choices=range(7),
                    help='选择要发送的字符串的索引 (0, 1, 2, 3, 4, 5， 6)')

# 解析命令行参数

args = parser.parse_args()

# 根据命令行参数选择字符串

message = strings[args.string_index]

# 串口配置

port = '/dev/ttysWK3'  # 串口号，根据您的设备进行更改
baudrate = 115200       # 波特率

# 发送字符串

def send_message(port, baudrate, message):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            # 将字符串编码为字节序列
            byte_message = message.encode('utf-8')
            # 发送字符串
            ser.write(byte_message)
            print(f"成功发送字符串: {message}")
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except Exception as e:
        print(f"发送过程中发生错误: {e}")

# 调用函数发送消息

send_message(port, baudrate, message)
```

首先需要通过scp将脚本文件上传到终端

```bash
scp mcu_control_uart.py nvidia@172.26.1.131:/home/nvidia
```

上传成功之后登录到终端，再执行下电指令

```bash
sudo python3 mcu-control-uart.py 3
```

运行结果如上图。
