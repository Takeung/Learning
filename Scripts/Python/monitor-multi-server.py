import os
import time
import threading
from datetime import datetime

# IP列表
ip_list = ['172.26.1.131', '172.26.1.163', 
           '172.26.1.187', '172.26.1.188', 
        # '172.26.1.190', '172.26.1.191', '172.26.1.192', '172.26.1.193',
           '192.168.11.244', '192.168.11.245', '192.168.11.246', '192.168.11.247', 
           '192.168.11.248', '192.168.11.249']
# 保存结果的文件
log_file = 'connection_log.txt'
# 全局变量，用于停止监控
stop_monitoring = False
# 文件写入的锁
file_lock = threading.Lock()

# 记录断开和恢复的时间
class ConnectionMonitor:
    def __init__(self, ip):
        self.ip = ip
        self.is_connected = True
        self.disconnection_count = 0
        self.disconnection_start_time = None

    def ping(self):
        # 使用os.system执行ping命令 (-c 1 表示发送1个包)
        response = os.system(f"ping -c 1 {self.ip} > /dev/null 2>&1")
        return response == 0  # 返回True表示ping成功，False表示失败

    def monitor(self):
        global stop_monitoring
        while not stop_monitoring:
            connected = self.ping()
            current_time = datetime.now()

            if self.is_connected and not connected:
                # 连接断开
                self.is_connected = False
                self.disconnection_start_time = current_time
                self.disconnection_count += 1
                print(f"{self.ip} 连接断开 at {current_time}")
            
            elif not self.is_connected and connected:
                # 连接恢复
                self.is_connected = True
                disconnection_duration = (current_time - self.disconnection_start_time).total_seconds()
                formatted_duration = self.format_duration(disconnection_duration)
                self.log_disconnection(self.disconnection_start_time, current_time, formatted_duration, disconnection_duration)
                print(f"{self.ip} 连接恢复 at {current_time}，断开时长 {formatted_duration}")
            
            time.sleep(1)  # 每秒检查一次

    def format_duration(self, duration):
        # 将秒数转换为天、小时、分钟、秒
        days, remainder = divmod(duration, 86400)  # 1天 = 86400秒
        hours, remainder = divmod(remainder, 3600) # 1小时 = 3600秒
        minutes, seconds = divmod(remainder, 60)   # 1分钟 = 60秒

        # 格式化输出
        if days > 0:
            return f"{int(days)}天 {int(hours)}小时 {int(minutes)}分 {int(seconds)}秒"
        elif hours > 0:
            return f"{int(hours)}小时 {int(minutes)}分 {int(seconds)}秒"
        elif minutes > 0:
            return f"{int(minutes)}分 {int(seconds)}秒"
        else:
            return f"{int(seconds)}秒"

    def log_disconnection(self, start_time, end_time, duration_str, duration):
        try:
            # 使用锁来同步文件写入操作，避免多线程冲突
            with file_lock:
                with open(log_file, 'a') as f:
                    f.write(f"{self.ip} 断开 at {start_time}, 恢复 at {end_time}, 持续时间: {duration_str}\n")
                    
                    # 增加对断开时间的判断
                    if 30 <= duration <= 60:
                        f.write(f"{self.ip} 疑似切区重启, 断开时长: {duration_str}\n")
                        print(f"{self.ip} 疑似切区重启, 断开时长: {duration_str}")

        except Exception as e:
            print(f"写入文件失败: {e}")


def start_monitoring(ip):
    monitor = ConnectionMonitor(ip)
    monitor.monitor()


# 启动多线程监控每个IP
threads = []
for ip in ip_list:
    thread = threading.Thread(target=start_monitoring, args=(ip,))
    thread.start()
    threads.append(thread)

# 主线程保持运行，并捕获用户的中断信号
try:
    while True:
        time.sleep(10)  # 主线程保持活动，10秒检查一次
except KeyboardInterrupt:
    print("\n用户中断，正在停止监控...")
    stop_monitoring = True

    # 等待所有线程结束
    for thread in threads:
        thread.join()

    print("所有IP监控已停止，程序已退出。")
