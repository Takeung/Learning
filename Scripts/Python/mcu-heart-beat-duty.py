import os
import time
import struct
import matplotlib.pyplot as plt
from select import poll, POLLIN

# 打开设备文件
fd = os.open("/dev/mcu-gpio2", os.O_RDWR)
if fd < 0:
    print("无法打开设备文件")
    exit()

print("成功打开设备文件")

# 设置poll对象
poller = poll()
poller.register(fd, POLLIN)

# 初始化数据存储
data_list = []
time_list = []
high_time = 0
low_time = 0
prev_value = None
start_time = time.time()

# 设置绘图
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [])
legend = ax.legend(["NO_FAULT_WAVE"], loc='upper right')

# 主循环
print("开始捕获数据...")

try:
    while True:
        events = poller.poll(3000)  # 3秒超时
        if len(events) == 0:
            print("超时，无数据")
        else:
            # 读取GPIO数据，每次读取2字节
            raw_data = os.read(fd, 2)
            if len(raw_data) == 2:
                # 解析数据为无符号16位整数
                data = struct.unpack('H', raw_data)[0]
                timestamp = time.time() - start_time

                # 存储数据和时间戳
                data_list.append(data)
                time_list.append(timestamp)

                # 保留最近两秒的数据
                while time_list and (timestamp - time_list[0] > 2):
                    time_list.pop(0)
                    data_list.pop(0)

                # 计算频率和占空比
                if len(data_list) > 1:
                    if prev_value is not None and prev_value != data:
                        if data == 1:
                            high_time = timestamp
                        elif data == 0:
                            low_time = timestamp

                    if high_time > low_time:  # 检测到上升沿
                        period = timestamp - low_time
                        frequency = 1 / period if period > 0 else 0
                        duty_cycle = (high_time - low_time) / period * 100 if period > 0 else 0

                        # 更新图例
                        if 9 <= frequency <= 11:
                            legend.get_texts()[0].set_text("MAJOR_FAULT_WAVE")
                        elif 4 <= frequency <= 6:
                            legend.get_texts()[0].set_text("MINOR_FAULT_WAVE")
                        elif 0.9 <= frequency <= 1.1:
                            legend.get_texts()[0].set_text("NO_FAULT_WAVE")

                        print(f"频率: {frequency:.2f} Hz, 占空比: {duty_cycle:.2f}%")

                prev_value = data

                # 绘制实时波形，仅显示最近两秒的数据
                ax.clear()
                ax.plot(time_list, data_list)
                ax.set(xlabel='时间 (s)', ylabel='GPIO 状态', title='GPIO 信号波形')
                ax.legend([legend.get_texts()[0].get_text()], loc='upper right')
                plt.draw()
                plt.pause(0.01)
            else:
                print(f"读取到的数据长度不正确: {len(raw_data)}")

except KeyboardInterrupt:
    print("\n捕获数据已停止")

# 关闭设备文件
os.close(fd)
print("设备文件已关闭")
