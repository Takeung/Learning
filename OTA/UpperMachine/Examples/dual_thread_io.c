#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>

#define BUFFER_SIZE 512

// 串口初始化函数
int init_serial(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8位数据位
    tty.c_iflag = IGNPAR;                           // 忽略奇偶校验错误
    tty.c_cflag |= (CLOCAL | CREAD);                // 启用接收器，忽略modem控制线
    tty.c_oflag = 0;                                // 无输出处理
    tty.c_lflag = 0;                                // 非规范模式（原始模式）

    tcflush(fd, TCIFLUSH);                          // 清空接收缓冲区
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

// 接收数据的线程
void *receive_thread(void *arg) {
    int fd = *(int *)arg;
    char buffer[BUFFER_SIZE];

    while (1) {
        memset(buffer, 0, sizeof(buffer));
        int recvBytes = read(fd, buffer, sizeof(buffer) - 1);
        if (recvBytes > 0) {
            printf("%.*s", recvBytes, buffer);
        } else if (recvBytes < 0) {
            perror("read");
            break;
        }
    }

    return NULL;
}

// 发送数据的线程
void *send_thread(void *arg) {
    int fd = *(int *)arg;
    char buffer[BUFFER_SIZE];

    while (1) {
        memset(buffer, 0, sizeof(buffer));
        printf("Enter message: ");
        fgets(buffer, sizeof(buffer), stdin);
        buffer[strcspn(buffer, "\n")] = '\0';  // 移除换行符

        if (write(fd, buffer, strlen(buffer)) < 0) {
            perror("write");
            break;
        }
    }

    return NULL;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <device> <baudrate>\n", argv[0]);
        return 1;
    }

    const char *device = argv[1];
    int baudrate = atoi(argv[2]);

    int fd = init_serial(device, baudrate);
    if (fd < 0) {
        return 1;
    }

    pthread_t recv_thread, snd_thread;

    // 启动接收线程
    if (pthread_create(&recv_thread, NULL, receive_thread, &fd) != 0) {
        perror("pthread_create");
        close(fd);
        return 1;
    }

    // 启动发送线程
    if (pthread_create(&snd_thread, NULL, send_thread, &fd) != 0) {
        perror("pthread_create");
        close(fd);
        return 1;
    }

    // 等待线程完成
    pthread_join(recv_thread, NULL);
    pthread_join(snd_thread, NULL);

    close(fd);
    return 0;
}
