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
    # response = "ACK: Message received"
    # sock.sendto(response.encode('utf-8'), address)
