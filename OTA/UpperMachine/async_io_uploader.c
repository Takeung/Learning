#define _GNU_SOURCE // 在源文件开头定义_GNU_SOURCE宏
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <termios.h>

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CRCCHR 'C'
#define BLOCK_SIZE 128
#define BUFFER_SIZE 512

#define CM0_BIN_NAME "cm0plus_app.bin"
#define CM7_0_BIN_NAME "cm7_0_app.bin"
#define CM7_1_BIN_NAME "cm7_1_app.bin"

static int fd;

// 声明初始化串口、发送文件和接收文件的函数
static int init_serial(const char *device, int baudrate);
static void io_handler(int sig, siginfo_t *info, void *context);
static void async_io_init(void);
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length);
static int send_file(int fd, const char *filename);
static int receive_file(const char *filename);
static void process_app_file(int fd);
static int process_flashing(int fd);

static void disable_async_io() {
    int flag = fcntl(fd, F_GETFL);   // 获取当前的文件状态标志
    flag &= ~O_ASYNC;                // 清除 O_ASYNC 标志，禁用异步 I/O
    fcntl(fd, F_SETFL, flag);        // 设置新的文件状态标志
}

static void enable_async_io() {
    int flag = fcntl(fd, F_GETFL);   // 获取当前的文件状态标志
    flag |= O_ASYNC;                 // 设置 O_ASYNC 标志，启用异步 I/O
    fcntl(fd, F_SETFL, flag);        // 设置新的文件状态标志
}

int main(int argc, char *argv[])
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <device> <baudrate>\n", argv[0]);
        return 1;
    }
    
    const char *device = argv[1];
    int baudrate = atoi(argv[2]);
    int res=0;

    while (1) {
        fd = init_serial(device, baudrate);
        if (fd < 0) {
            return 1;
        }
        async_io_init();
        if (1 == process_flashing(fd)) {
            break;
        }
        close(fd);
        sleep(6);
    }

    return res;
}

static int init_serial(const char *device, int baudrate) {
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
    // 设置波特率
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    // // 设置字符大小
    // tty.c_cflag &= ~CSIZE;
    // tty.c_cflag |= CS8;

    // // 设置无校验位
    // tty.c_cflag &= ~PARENB;

    // // 设置一位停止位
    // tty.c_cflag &= ~CSTOPB;

    // // 设置本地模式和允许读取
    // tty.c_cflag |= (CLOCAL | CREAD);

    // // 关闭硬件流控（RTS/CTS）
    // tty.c_cflag &= ~CRTSCTS;

    // // 关闭软件流控
    // tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR);

    // tty.c_oflag &= ~(ONLCR| OPOST );

    // // 设置为原始模式（非规范模式）
    // tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tty.c_iflag = 0x1;
    tty.c_oflag = 0x0;
    tty.c_cflag = 0x18b2;
    tty.c_lflag = 0x0;

    // 设置非规范模式下的控制字符
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    tcflush(fd,TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0;
    while (length--) {
        crc ^= *data++ << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

static int send_file(int fd, const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("fopen");
        return -1;
    }

    uint8_t block_number = 1;
    uint8_t buffer[BLOCK_SIZE];
    uint8_t packet[BLOCK_SIZE + 5];

    // Wait for NAK from receiver
    char ch;
    do {
        if (read(fd, &ch, 1) < 0) {
            perror("read");
            fclose(file);
            return -1;
        }
    } while (ch != CRCCHR);

    while (1) {
        size_t bytes_read = fread(buffer, 1, BLOCK_SIZE, file);
        if (bytes_read == 0) {
            break; // No more data to send
        }

        packet[0] = SOH;
        packet[1] = block_number;
        packet[2] = ~block_number;
        memcpy(&packet[3], buffer, bytes_read);

        if (bytes_read < BLOCK_SIZE) {
            memset(&packet[3 + bytes_read], 0x1A, BLOCK_SIZE - bytes_read); // Padding with 0x1A
        }

        uint16_t crc = crc16_ccitt(&packet[3], BLOCK_SIZE);
        packet[BLOCK_SIZE + 3] = crc >> 8;
        packet[BLOCK_SIZE + 4] = crc & 0xFF;

        if (write(fd, packet, BLOCK_SIZE + 5) < 0) {
            perror("write");
            fclose(file);
            return -1;
        }

        if (read(fd, &ch, 1) < 0) {
            perror("read");
            fclose(file);
            return -1;
        }

        if (ch == ACK) {
            block_number++;
        } else if (ch == NAK) {
            fseek(file, -BLOCK_SIZE, SEEK_CUR); // Resend the current block
        } else {
            fprintf(stderr, "Unexpected response: %c\n", ch);
            fclose(file);
            return -1;
        }
    }
    char eot = EOT;
    // Send EOT
    while (1) {
        if (write(fd, &eot, 1) < 0) {
            perror("write");
            fclose(file);
            return -1;
        }

        if (read(fd, &ch, 1) < 0) {
            perror("read");
            fclose(file);
            return -1;
        }

        if (ch == ACK) {
            break;
        }
    }

    fclose(file);
    return 0;
}

static int receive_file(const char *filename) {
    FILE *file = fopen(filename, "wb");
    if (!file) {
        perror("fopen");
        return -1;
    }

    uint8_t block_number = 1;
    uint8_t packet[BLOCK_SIZE + 5];
    char ch = CRCCHR;

    if (write(fd, &ch, 1) < 0) {
        perror("write");
        fclose(file);
        return -1;
    }

    while (1) {
        if (read(fd, packet, BLOCK_SIZE + 5) < 0) {
            perror("read");
            fclose(file);
            return -1;
        }

        if (packet[0] == EOT) {
            ch = ACK;
            if (write(fd, &ch, 1) < 0) {
                perror("write");
                fclose(file);
                return -1;
            }
            break;
        }

        if (packet[0] != SOH || packet[1] != block_number || packet[2] != (uint8_t)~block_number) {
            ch = NAK;
            if (write(fd, &ch, 1) < 0) {
                perror("write");
                fclose(file);
                return -1;
            }
            continue;
        }

        uint16_t crc_received = (packet[BLOCK_SIZE + 3] << 8) | packet[BLOCK_SIZE + 4];
        uint16_t crc_calculated = crc16_ccitt(&packet[3], BLOCK_SIZE);

        if (crc_received != crc_calculated) {
            ch = NAK;
            if (write(fd, &ch, 1) < 0) {
                perror("write");
                fclose(file);
                return -1;
            }
            continue;
        }

        if (fwrite(&packet[3], 1, BLOCK_SIZE, file) < BLOCK_SIZE) {
            perror("fwrite");
            fclose(file);
            return -1;
        }

        ch = ACK;
        if (write(fd, &ch, 1) < 0) {
            perror("write");
            fclose(file);
            return -1;
        }

        block_number++;
    }

    fclose(file);
    return 0;
}

static void io_handler(int sig, siginfo_t *info, void *context)
{
    unsigned char buffer[BUFFER_SIZE] = {0};

    if (SIGRTMIN != sig)
    {
        return;
    }

    if (POLL_IN == info->si_code)
    {
        memset(buffer, 0, BUFFER_SIZE);
        int recvBytes = read(fd, buffer, BUFFER_SIZE - 1);
        if (recvBytes > 0) {
            printf("%.*s", recvBytes, buffer);
        } else if (recvBytes < 0) {
            perror("read");
        }
    }
}

/*异步IO初始化函数*/
static void async_io_init(void)
{
    struct sigaction sig;
    int flag;

    /*使能异步I/O*/            /**/
    flag = fcntl(fd, F_GETFL); /*使能串口的异步I/O功能*/
    flag |= O_ASYNC;
    fcntl(fd, F_SETFL, flag);
    /*设置异步I/O的所有者*/
    fcntl(fd, F_SETOWN, getpid());
    /*为实时信号SIGRTMIN作为异步I/O信号*/
    fcntl(fd, F_SETSIG, SIGRTMIN);

    sig.sa_sigaction = io_handler;
    sig.sa_flags = SA_SIGINFO;
    sigisemptyset(&sig.sa_mask);
    sigaction(SIGRTMIN, &sig, NULL);
}

static void process_app_file(int fd) {
    char input[4] = {0};

    // 进入OTA模式
    printf("Enter OTA mode? (yes/no): \r\n");
    while (1) {
        fgets(input, sizeof(input), stdin);
        input[strcspn(input, "\n")] = 0; // 移除输入中的换行符

        if (strcmp(input, "yes") == 0) {
            // 发送数据
            if (write(fd, "^^0109050000000D$$", 18) < 0) {
                perror("Enter OTA mode Error");
            } else {
                printf("Enter OTA mode!\r\n");
            }
            memset(input, 0, sizeof(input));
            break; // 继续下一个数据发送
        } else if (strcmp(input, "no") == 0) {
            printf("Send aborted.\n");
            return; // 停止后续发送
        } else if (strlen(input) > 0) {
            printf("Enter OTA: Invalid input. Please type 'yes' or 'no': ");
        } else {
            // do nothing
        }
        memset(input, 0, sizeof(input));
    }

    // 刷写cm0
    sleep(10);
    printf("Start flashing for CM0? (yes/no/next): \r\n");
    while (1) {
        fgets(input, sizeof(input), stdin);
        input[strcspn(input, "\n")] = 0; // 移除输入中的换行符
        printf("CM0 input is: %s", input);

        disable_async_io();

        if (strcmp(input, "yes") == 0) {
            if (write(fd, "1", 1) < 0) {
                perror("Write CM0 Error");
            }
            sleep(3);
            if (0 == send_file(fd, CM0_BIN_NAME)) {
                printf("Send CM0 App OK\r\n");
                memset(input, 0, sizeof(input));
                break; // 继续下一个数据发送
            } else {
                printf("Retry flashing for CM0? (yes/no/next): \r\n");
            }
            sleep(1);
        } else if (strcmp(input, "next") == 0) {
            memset(input, 0, sizeof(input));
            break; // 继续下一个数据发送
        } else if (strcmp(input, "no") == 0) {
            printf("Send aborted.\r\n");
            return; // 停止后续发送
        } else if (strlen(input) > 0) {
            printf("Flashing for CM0: Invalid input. Please type 'yes/no/next': \r\n");
        } else {
            // do nothing
        }

        enable_async_io();

        memset(input, 0, sizeof(input));
    }

    // 刷写cm7_0
    sleep(3);
    printf("Start flashing for CM7_0? (yes/no/next): \r\n");
    while (1) {
        fgets(input, sizeof(input), stdin);
        input[strcspn(input, "\n")] = 0; // 移除输入中的换行符
        printf("CM7_0 input is: %s", input);

        disable_async_io();

        if (strcmp(input, "yes") == 0) {
            if (write(fd, "2", 1) < 0) {
                perror("Write CM7_0 Error");
            }
            sleep(3);
            if (0 == send_file(fd, CM7_0_BIN_NAME)) {
                printf("Send CM7_0 App OK\r\n");
                memset(input, 0, sizeof(input));
                break; // 继续下一个数据发送
            } else {
                printf("Retry flashing for CM7_0? (yes/no/next): \r\n");
            }
            sleep(1);
        } else if (strcmp(input, "next") == 0) {
            memset(input, 0, sizeof(input));
            break; // 继续下一个数据发送
        } else if (strcmp(input, "no") == 0) {
            printf("Send aborted.\r\n");
            return; // 停止后续发送
        } else if (strlen(input) > 0) {
            printf("Flashing for CM7_0: Invalid input. Please type 'yes/no/next': \r\n");
        } else {
            // do nothing
        }

        enable_async_io();

        memset(input, 0, sizeof(input));
    }

    // 刷写cm7_1
    sleep(3);
    printf("Start flashing for CM7_1? (yes/no/next): \r\n");
    while (1) {
        fgets(input, sizeof(input), stdin);
        input[strcspn(input, "\n")] = 0; // 移除输入中的换行符
        printf("CM7_1 input is: %s", input);

        disable_async_io();

        if (strcmp(input, "yes") == 0) {
            if (write(fd, "3", 1) < 0) {
                perror("Write CM7_1 Error");
            }
            sleep(3);
            if (0 == send_file(fd, CM7_1_BIN_NAME)) {
                printf("Send CM7_1 App OK\r\n");
                memset(input, 0, sizeof(input));
                break; // 继续下一个数据发送
            } else {
                printf("Retry flashing for CM7_1? (yes/no/next): \r\n");
            }
            sleep(1);
        } else if (strcmp(input, "next") == 0) {
            memset(input, 0, sizeof(input));
            break; // 继续下一个数据发送
        } else if (strcmp(input, "no") == 0) {
            printf("Send aborted.\r\n");
            return; // 停止后续发送
        } else if (strlen(input) > 0) {
            printf("Flashing for CM7_1: Invalid input. Please type 'yes/no/next': \r\n");
        } else {
            // do nothing
        }

        enable_async_io();

        memset(input, 0, sizeof(input));
    }

    // 切换分区
    sleep(3);
    printf("Switch workflash? (yes/no): \r\n");
    while (1) {
        fgets(input, sizeof(input), stdin);
        input[strcspn(input, "\n")] = 0; // 移除输入中的换行符

        if (strcmp(input, "yes") == 0) {
            if (write(fd, "4", 1) < 0) {
                perror("Switch Workflash Error");
            }
            sleep(1);
            memset(input, 0, sizeof(input));
            break; // 继续下一个数据发送
        } else if (strcmp(input, "no") == 0) {
            printf("Stay in current workflash. Waiting for reboot.\r\n");
            return; // 停止后续发送
        } else if (strlen(input) > 0) {
            printf("Switch workflash: Invalid input. Please type 'yes' or 'no': \r\n");
        } else {
            // do nothing
        }
        memset(input, 0, sizeof(input));
    }
}

static int process_flashing(int fd) {
    int ret = 0;
    char input_cmd;

    while ((input_cmd - '0' < 0) || (input_cmd - '0' > 5)) {
        printf("\r\nEnter OTA(0) or Flashing for Core(1/2/3) or Switching(4) or Quit(5)?\r\n");
        scanf(" %c", &input_cmd);  // 读取用户输入的字符
        printf("Input CMD is: %c\r\n", input_cmd);
    }

    disable_async_io();

    switch (input_cmd) {
        case '0':
            if (write(fd, "^^0109050000000D$$", 18) < 0) {
                perror("Enter OTA mode Error");
            } else {
                printf("Send CMD for OTA OK!\r\n");
            }
            sleep(3);
            break;
        case '1':
            if (write(fd, &input_cmd, 1) < 0) {
                perror("Write CM0 Error");
                return -1;
            }
            sleep(3);
            printf("Send CM0 App Start\r\n");
            ret = send_file(fd, CM0_BIN_NAME);
            if (ret == 0) {
                printf("Send CM0 App and Flash OK\r\n");
            }
            break;
        case '2':
            if (write(fd, &input_cmd, 1) < 0) {
                perror("Write CM7_0 Error");
                return -1;
            }
            sleep(3);
            printf("Send CM7_0 App Start\r\n");
            ret = send_file(fd, CM7_0_BIN_NAME);
            if (ret == 0) {
                printf("Send CM7_0 App and Flash OK\r\n");
            }
            break;
        case '3':
            if (write(fd, &input_cmd, 1) < 0) {
                perror("Write CM7_1 Error");
                return -1;
            }
            sleep(3);
            printf("Send CM7_1 App Start\r\n");
            ret = send_file(fd, CM7_1_BIN_NAME);
            if (ret == 0) {
                printf("Send CM7_1 App and Flash OK\r\n");
            }
            break;
        case '4':
            if (write(fd, &input_cmd, 1) < 0) {
                perror("Switch Workflash Error");
                return -1;
            } else {
                printf("Switch Workflash Start. Waiting for Reboot!\r\n");
            }
            break;
        case '5':
            printf("Quit from Upper Machine!\r\n");
            return 1;
        default:
            printf("Please input 1-4 at last\r\n");
    }
    
    enable_async_io();
    
    sleep(3);

    return ret;
}