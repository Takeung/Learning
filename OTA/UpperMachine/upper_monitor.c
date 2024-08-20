#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <pthread.h>

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CRCCHR 'C'
#define BLOCK_SIZE 128

static int serialFd;

// 线程函数，用于持续读取串口数据
void* read_serial_thread();

// 声明初始化串口、发送文件和接收文件的函数
void init_serial(const char *device, int baudrate);
int send_file(const char *filename);
int receive_file(const char *filename);

int main(int argc, char *argv[]) {
    // if (argc != 6) {
    //     fprintf(stderr, "Usage: %s <device> <baudrate> <send|receive> <file>\n", argv[0]);
    //     return 1;
    // }
    
    // const char *device = argv[1];
    // int baudrate = atoi(argv[2]);
    // const char *mode = argv[3];
    // const char *filename = argv[4];
    // uint8_t   cmd_int = atoi(argv[5]);
    // char      cmd     = argv[5][0];
    int res = 0;

    // printf("before init_serial\n");
    // int serialFd = init_serial(device, baudrate);
    init_serial("/dev/ttysWK3", 115200);
    printf("serialFd is: %d\r\n", serialFd);
    if (serialFd < 0) {
        return 1;
    }
    
    uint8_t buffer[256];
    ssize_t n;

    while (1) {
        // 接收数据
        char buffer[255];
        int recvBytes= read(serialFd , buffer, sizeof(buffer));
        printf("Received: %.*s\n", recvBytes, buffer);
        sleep(1);
    }

    // 创建线程以持续读取串口数据
    // pthread_t read_thread;
    // if (pthread_create(&read_thread, NULL, read_serial_thread, (void*)(intptr_t)serialFd) != 0) {
    //     perror("pthread_create");
    //     close(serialFd);
    //     return 1;
    // }

    // switch(cmd_int){
    //     case 1:
    //     case 2:
    //     case 3:
    //         if (write(serialFd, &cmd, 1) < 0) {
    //             perror("write");
    //             close(serialFd);
    //             return -1;
    //         }
    //         printf("send cmd ok\n");
    //         sleep(3);
            
    //         if (strcmp(mode, "send") == 0) {
    //             res = send_file(serialFd, filename);
    //         } else if (strcmp(mode, "receive") == 0) {
    //             res = receive_file(serialFd, filename);
    //         } else {
    //             fprintf(stderr, "Unknown mode: %s\n", mode);
    //             close(serialFd);
    //             return 1;
    //         }
    //         break;
    //     case 4:
    //         if (write(serialFd, &cmd, 1) < 0) {
    //             perror("write");
    //             close(serialFd);
    //             return -1;
    //         }
    //         break;
    //     default:
    //         printf("please input 1-4 at last");
    // }

    // 等待读取线程完成
    // pthread_join(read_thread, NULL);

    close(serialFd);
    return res;
}

// 读取串口数据并打印的线程函数
void* read_serial_thread() {
    uint8_t buffer[256];
    ssize_t n;

    while (1) {
        n = read(serialFd, buffer, sizeof(buffer) - 1);
        printf("Buffer length: %ld\r\n", n);
        if (n < 0) {
            if (errno != EAGAIN) {
                perror("read");
            }
            continue;
        }
        if (n == 0) {
            // EOF or no more data, exit the thread
            break;
        }
        buffer[n] = '\0'; // Null-terminate the buffer
        for (int i = 0; i < n; i++) {
            printf("Received data: %c\n", buffer[i]);
        }
    }

    pthread_exit(NULL);
}

// 初始化串口函数，设置非阻塞模式
void init_serial(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (serialFd < 0) {
        perror("open");
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serialFd, &tty) != 0) {
        perror("tcgetattr");
        close(serialFd);
    }

    // 设置波特率
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    // 设置字符大小
    // tty.c_cflag &= ~CSIZE;
    // tty.c_cflag |= CS8;

    // 设置无校验位
    // tty.c_cflag &= ~PARENB;

    // 设置一位停止位
    // tty.c_cflag &= ~CSTOPB;

    // 设置本地模式和允许读取
    // tty.c_cflag |= (CLOCAL | CREAD);

    // 关闭硬件流控（RTS/CTS）
    // tty.c_cflag &= ~CRTSCTS;

    // 关闭软件流控
    // tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR);
    // tty.c_oflag &= ~(ONLCR| OPOST );

    // 设置为原始模式（非规范模式）
    // tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    tty.c_iflag = 0x1;
    tty.c_oflag = 0x0;
    tty.c_cflag = 0x18b2;
    tty.c_lflag = 0x0;

    // 设置非规范模式下的控制字符
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    tcflush(serialFd, TCIFLUSH);

    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(serialFd);
    }
}

uint16_t crc16_ccitt(const uint8_t *data, uint16_t length) {
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

// 发送文件函数支持非阻塞接收
int send_file(const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("fopen");
        return -1;
    }

    uint8_t block_number = 1;
    uint8_t buffer[BLOCK_SIZE];
    uint8_t packet[BLOCK_SIZE + 5];
    char ch;
    
    // 等待接收方的 NAK
    do {
        if (read(serialFd, &ch, 1) < 0) {
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

        if (write(serialFd, packet, BLOCK_SIZE + 5) < 0) {
            perror("write");
            fclose(file);
            return -1;
        }

        if (read(serialFd, &ch, 1) < 0) {
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
        if (write(serialFd, &eot, 1) < 0) {
            perror("write");
            fclose(file);
            return -1;
        }

        if (read(serialFd, &ch, 1) < 0) {
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

// 接收文件函数
int receive_file(const char *filename) {
    FILE *file = fopen(filename, "wb");
    if (!file) {
        perror("fopen");
        return -1;
    }

    uint8_t block_number = 1;
    uint8_t packet[BLOCK_SIZE + 5];
    char ch = CRCCHR;

    if (write(serialFd, &ch, 1) < 0) {
        perror("write");
        fclose(file);
        return -1;
    }

    while (1) {
        if (read(serialFd, packet, BLOCK_SIZE + 5) < 0) {
            perror("read");
            fclose(file);
            return -1;
        }

        if (packet[0] == EOT) {
            ch = ACK;
            if (write(serialFd, &ch, 1) < 0) {
                perror("write");
                fclose(file);
                return -1;
            }
            break;
        }

        if (packet[0] != SOH || packet[1] != block_number || packet[2] != (uint8_t)~block_number) {
            ch = NAK;
            if (write(serialFd, &ch, 1) < 0) {
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
            if (write(serialFd, &ch, 1) < 0) {
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
        if (write(serialFd, &ch, 1) < 0) {
            perror("write");
            fclose(file);
            return -1;
        }

        block_number++;
    }

    fclose(file);
    return 0;
}
