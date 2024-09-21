#!/usr/bin/env python3
import socket
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
parser = argparse.ArgumentParser(description='通过串口或UDP发送指定字符串')
parser.add_argument('method', type=str, choices=['uart', 'udp'],
                    help='选择发送方式：uart 或 udp')
parser.add_argument('string_index', type=int, choices=range(7),
                    help='选择要发送的字符串的索引 (0, 1, 2, 3, 4, 5, 6)')

# 解析命令行参数
args = parser.parse_args()

# 根据命令行参数选择字符串
message = strings[args.string_index]

# UDP配置
src_port = 13137   # 发送方端口
dst_ip = "192.168.11.110"
dst_port = 13137   # 接收方端口

# UART配置
uart_port = '/dev/ttysWK3'  # 串口号，根据您的设备进行更改
baudrate = 115200           # 波特率

def send_udp_message(message, src_port, dst_ip, dst_port):
    # 创建UDP套接字，并绑定到指定的发送方端口
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 绑定到发送方端口
    sock.bind(('', src_port))
    
    try:
        # 发送UDP报文
        sock.sendto(message.encode(), (dst_ip, dst_port))
        print(f"Message sent via UDP from port {src_port} to {dst_ip}:{dst_port}")
    except Exception as e:
        print(f"Failed to send UDP message: {e}")
    finally:
        # 关闭套接字
        sock.close()

def send_uart_message(port, baudrate, message):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            # 将字符串编码为字节序列
            byte_message = message.encode('utf-8')
            # 发送字符串
            ser.write(byte_message)
            print(f"Message sent via UART: {message}")
    except serial.SerialException as e:
        print(f"UART error: {e}")
    except Exception as e:
        print(f"Error during UART sending: {e}")

# 根据用户选择的方式发送消息
if args.method == 'udp':
    send_udp_message(message, src_port, dst_ip, dst_port)
elif args.method == 'uart':
    send_uart_message(uart_port, baudrate, message)
