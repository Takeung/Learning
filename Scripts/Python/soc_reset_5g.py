import time

def modify_file_value(file_path):
    try:
        # 读取文件内容
        with open(file_path, 'r') as file:
            content = file.read().strip()

        # 检查文件中的值是否为0或1
        if content not in ['0', '1']:
            print(f"文件内容无效: {content}")
            return

        # 如果文件内容是0，将其改为1
        if content == '0':
            with open(file_path, 'w') as file:
                file.write('1')
        else:
            with open(file_path, 'w') as file:
                file.write('0')

    except FileNotFoundError:
        print(f"找不到文件: {file_path}")
    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    modify_file_value('/sys/class/tz_gpio/5g1-reset/value')
    # 延时2.5秒
    time.sleep(2.5)
    modify_file_value('/sys/class/tz_gpio/5g1-reset/value')