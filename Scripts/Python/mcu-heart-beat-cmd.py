import subprocess
import time
import numpy as np
import argparse

# 初始化数据存储
window_size = 3  # 窗口大小，单位秒

def monitor_signal(file_path):
    global process
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
                print(f"current_time: {current_time}, signal_value: {signal_value}")
                
                # 移除超过窗口大小的旧数据
                times = [t for t in times if t >= current_time - window_size]
                values = values[-len(times):]
                
                # 计算频率和占空比
                if len(values) > 1:
                    # 查找上升沿和下降沿
                    rising_edges = np.where(np.array(values) == 1)[0]  # 接收到1的时刻为上升沿
                    falling_edges = np.where(np.array(values) == 0)[0]  # 接收到0的时刻为下降沿

                    if len(rising_edges) > 1 and len(falling_edges) > 0:
                        # 当前周期：两个相邻上升沿之间的时间差
                        last_rising_edge = rising_edges[-1]
                        previous_rising_edge = rising_edges[-2]
                        period_time = times[last_rising_edge] - times[previous_rising_edge]
                        
                        # 查找最近的下降沿
                        valid_falling_edges = falling_edges[falling_edges > previous_rising_edge]
                        
                        if len(valid_falling_edges) > 0:
                            # 下降沿时刻
                            falling_time = times[valid_falling_edges[0]]
                            
                            # 上升沿时刻
                            rising_time = times[previous_rising_edge]
                            
                            # 计算占空比：下降沿时刻 - 上升沿时刻 / 周期时间
                            duty_cycle = ((falling_time - rising_time) / period_time) * 100
                            
                            # 计算频率
                            frequency = 1 / period_time
                            print(f"falling_time: {falling_time}, rising_time: {rising_time}")
                            
                            # 打印频率和占空比
                            print(f'Frequency: {frequency:.2f} Hz, Duty Cycle: {duty_cycle:.2f}%')
                    else:
                        print('Frequency: N/A, Duty Cycle: N/A')
    
    except KeyboardInterrupt:
        print("Monitoring stopped by user.")
    finally:
        process.terminate()
        process.wait()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Monitor GPIO signal and print waveform frequency and duty cycle.')
    parser.add_argument('gpio_path', type=str, help='Path to the GPIO device (e.g., /dev/mcu-gpio4)')
    args = parser.parse_args()
    
    monitor_signal(args.gpio_path)
