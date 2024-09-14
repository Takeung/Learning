#!/usr/bin/env python3
import socket
import serial
import subprocess
import time
import numpy as np
import argparse

# 定义要发送的字符串列表
strings = [
    "^^01010500000005$$",
    "^^01020500000006$$",
    "^^01030500000007$$",
    "^^01040500000000$$",
    "^^01050500000001$$",
    "^^01060500000002$$",
    "^^01070500000003$$",
    "^^0109050000000D$$"
]

def print_command_menu():
    print('选择0:给MCU下电\n选择1:给ORIN下电\n选择2:给Nx下电\n选择3:开补光灯\n选择4:关补光灯\n选择5:给毫米波雷达下电\n选择6:给摄像头下电\n选择7:OTA升级')

# UDP配置
src_port = 13137   # 发送方端口
dst_ip = "192.168.11.110"
dst_port = 13137   # 接收方端口

# UART配置
uart_port = '/dev/ttysWK3'  # 串口号，根据您的设备进行更改
baudrate = 115200           # 波特率

# UDP 发送函数
def send_udp_message(message, src_port, dst_ip, dst_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', src_port))
    
    try:
        sock.sendto(message.encode(), (dst_ip, dst_port))
        print(f"Message sent via UDP from port {src_port} to {dst_ip}:{dst_port}")
    except Exception as e:
        print(f"Failed to send UDP message: {e}")
    finally:
        sock.close()

# UART 发送函数
def send_uart_message(port, baudrate, message):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            byte_message = message.encode('utf-8')
            ser.write(byte_message)
            print(f"Message sent via UART: {message}")
    except serial.SerialException as e:
        print(f"UART error: {e}")
    except Exception as e:
        print(f"Error during UART sending: {e}")

# GPIO 信号监控函数
def monitor_signal(file_path):
    window_size = 3  # 窗口大小，单位秒
    process = subprocess.Popen(['cat', file_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1, universal_newlines=True)
    
    times = []
    values = []
    
    try:
        while True:
            output = process.stdout.readline().strip()

            if output:
                signal_value = int(output)
                current_time = time.time()
                times.append(current_time)
                values.append(signal_value)
                
                times = [t for t in times if t >= current_time - window_size]
                values = values[-len(times):]
                
                if len(values) > 1:
                    rising_edges = np.where(np.array(values) == 1)[0]
                    falling_edges = np.where(np.array(values) == 0)[0]

                    if len(rising_edges) > 1 and len(falling_edges) > 0:
                        last_rising_edge = rising_edges[-1]
                        previous_rising_edge = rising_edges[-2]
                        period_time = times[last_rising_edge] - times[previous_rising_edge]
                        
                        valid_falling_edges = falling_edges[falling_edges > previous_rising_edge]
                        
                        if len(valid_falling_edges) > 0:
                            falling_time = times[valid_falling_edges[0]]
                            rising_time = times[previous_rising_edge]
                            duty_cycle = ((falling_time - rising_time) / period_time) * 100
                            frequency = 1 / period_time
                            print(f'Frequency: {frequency:.2f} Hz, Duty Cycle: {duty_cycle:.2f}%')
                    else:
                        print('Frequency: N/A, Duty Cycle: N/A')
    
    except KeyboardInterrupt:
        print("Monitoring stopped by user.")
    finally:
        process.terminate()
        process.wait()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='通过串口/UDP发送字符串或监控GPIO信号。')
    parser.add_argument('mode', type=str, choices=['send', 'monitor'], help='选择操作模式：send 发送消息，monitor 监控GPIO信号')
    
    # 如果选择发送消息，则需要额外的参数
    parser.add_argument('-m', '--method', type=str, choices=['uart', 'udp'], help='选择发送方式：uart 或 udp')
    parser.add_argument('-s', '--string_index', type=int, choices=range(7), help='选择要发送的字符串的索引 (0, 1, 2, 3, 4, 5, 6)')
    
    # 如果选择监控GPIO信号，则需要传入设备路径
    parser.add_argument('-g', '--gpio_path', type=str, help='路径到GPIO设备 (如: /dev/mcu-gpio4)')
    
    args = parser.parse_args()

    if args.mode == 'send':
        print_command_menu()
        if args.method is None or args.string_index is None:
            print("发送模式需要 --method 和 --string_index 参数。")
        else:
            message = strings[args.string_index]
            if args.method == 'udp':
                send_udp_message(message, src_port, dst_ip, dst_port)
            elif args.method == 'uart':
                send_uart_message(uart_port, baudrate, message)
    elif args.mode == 'monitor':
        if args.gpio_path is None:
            print("监控模式需要 --gpio_path 参数。")
        else:
            monitor_signal(args.gpio_path)
