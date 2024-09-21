#!/usr/bin/env python3
import socket
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

# socket配置
src_port = 13137   # 发送方端口
dst_ip = "192.168.11.110"
dst_port = 13137   # 接收方端口

def send_udp_message(message, src_port, dst_ip, dst_port):
    # 创建UDP套接字，并绑定到指定的发送方端口
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 绑定到发送方端口
    sock.bind(('', src_port))
    
    try:
        # 发送UDP报文
        sock.sendto(message.encode(), (dst_ip, dst_port))
        print(f"Message sent from port {src_port} to {dst_ip}:{dst_port}")
    except Exception as e:
        print(f"Failed to send message: {e}")
    finally:
        # 关闭套接字
        sock.close()

# 示例使用

# 调用函数发送消息
send_udp_message(message, src_port, dst_ip, dst_port)