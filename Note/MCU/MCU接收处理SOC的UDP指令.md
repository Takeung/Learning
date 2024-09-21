# MCU接受处理SOC的UDP指令

MCU接收SOC指令

```python
#!/usr/bin/env python3
import socket

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
ip = "192.168.11.110"
port = 5005

def send_udp_message(message, ip, port):
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # 发送UDP报文
        sock.sendto(message.encode(), (ip, port))
        print(f"Message sent to {ip}:{port}")
    except Exception as e:
        print(f"Failed to send message: {e}")
    finally:
        # 关闭套接字
        sock.close()

# 调用函数发送消息
send_udp_message(message, ip, port)
```

SOC持续监听UDP端口13137

```python
import socket

# 定义服务器地址和端口
server_address = ('0.0.0.0', 13137)  # 监听所有可用的网络接口

# 创建UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 绑定服务器地址
sock.bind(server_address)

print(f"Listening on UDP port {server_address[1]}...")

# 持续监听并接收数据
while True:
    data, address = sock.recvfrom(4096)  # 最大接收4096字节
    print(f"Received {len(data)} bytes from {address}")
    print(f"Data: {data.decode('utf-8')}")  # 将数据解码为字符串

    # 可选：发送回应给客户端
    response = "ACK: Message received"
    sock.sendto(response.encode('utf-8'), address)
```

