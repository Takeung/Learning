import subprocess
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse

# 初始化数据存储
sampling_interval = 0.001  # 采样间隔，单位秒
window_size = 3  # 窗口大小，单位秒

def update_plot(frame, line, times, values, text_obj, legend_obj, ax):
    output = process.stdout.readline().strip()
    
    if output:
        signal_value = int(output)
        current_time = time.time()

        times.append(current_time)
        values.append(signal_value)
        
        # 移除超过窗口大小的旧数据
        times = [t for t in times if t >= current_time - window_size]
        values = values[-len(times):]
        
        # 更新线条数据
        line.set_data(times, values)
        
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
                    
                    # 更新频率和占空比的文本对象
                    text_obj.set_text(f'Frequency: {frequency:.2f} Hz\nDuty Cycle: {duty_cycle:.2f}%')
                    
                    # 更新图例文本
                    if frequency >= 9 and frequency <= 11:
                        legend_text = 'MAJOR_FAULT_WAVE'
                    elif frequency >= 4 and frequency <= 6:
                        legend_text = 'MINOR_FAULT_WAVE'
                    elif frequency >= 0.9 and frequency <= 1.1:
                        legend_text = 'NO_FAULT_WAVE'
                    else:
                        legend_text = 'UNKNOWN_FAULT_WAVE'
                        
                    legend_obj.set_text(legend_text)
            else:
                text_obj.set_text('Frequency: N/A\nDuty Cycle: N/A')
                legend_obj.set_text('UNKNOWN_FAULT_WAVE')
    
    # 调整x轴范围以显示最新数据
    ax.set_xlim(max(0, current_time - window_size), current_time)
    
    # 确保y轴范围适应矩形波
    ax.set_ylim(-0.1, 1.1)
    
    return line, text_obj, legend_obj

def start_monitoring(file_path):
    global process
    process = subprocess.Popen(['cat', file_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1, universal_newlines=True)
    
    fig, ax = plt.subplots()
    ax.set_xlim(0, window_size)
    ax.set_ylim(-0.1, 1.1)
    
    # 初始化矩形波线条
    times = []
    values = []
    line, = ax.step([], [], where='post', lw=2)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Signal Value')
    ax.set_title('Heartbeat Signal Waveform')
    
    # 添加动态更新的文本对象
    text_obj = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12,
                       verticalalignment='top', bbox=dict(boxstyle='round,pad=0.3', edgecolor='black', facecolor='white'))
    
    # 添加动态更新的图例对象，放置在右上角，距离右侧边框0.05单位
    legend_obj = ax.text(0.95, 0.95, '', transform=ax.transAxes, fontsize=12,
                         verticalalignment='top', horizontalalignment='right',
                         bbox=dict(boxstyle='round,pad=0.3', edgecolor='black', facecolor='lightgrey'))
    
    # 动画更新函数
    ani = animation.FuncAnimation(fig, update_plot, fargs=(line, times, values, text_obj, legend_obj, ax),
                                  interval=sampling_interval*1000, blit=True)
    
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Monitor GPIO signal and plot waveform.')
    parser.add_argument('gpio_path', type=str, help='Path to the GPIO device (e.g., /dev/mcu-gpio4)')
    args = parser.parse_args()
    
    try:
        start_monitoring(args.gpio_path)
    except KeyboardInterrupt:
        print("Monitoring stopped by user.")
    finally:
        process.terminate()
        process.wait()
