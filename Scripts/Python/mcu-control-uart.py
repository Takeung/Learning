#!/usr/bin/env python3
import subprocess
import os
import sys
import time
import serial
import argparse

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
print('选择0:给Nx单独下电指令\n选择1:给Nx单独上电指令\n选择2:给ORIN下电指令\n选择3:给MCU下电指令\n选择4:给毫米波雷达下电指令\n选择5:给摄像头下电指令\n选择6:OTA升级')

# 设置argparse
parser = argparse.ArgumentParser(description='通过串口发送指定字符串')
parser.add_argument('string_index', type=int, choices=range(7),
                    help='选择要发送的字符串的索引 (0, 1, 2, 3, 4, 5, 6)')

# 解析命令行参数
args = parser.parse_args()

# 根据命令行参数选择字符串
message = strings[args.string_index]

# 串口配置
port = '/dev/ttysWK3'  # 串口号，根据您的设备进行更改
baudrate = 115200       # 波特率

# 发送字符串
def send_uart_message(port, baudrate, message):
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
send_uart_message(port, baudrate, message)