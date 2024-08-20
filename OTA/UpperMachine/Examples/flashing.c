#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CRCCHR 'C'
#define BLOCK_SIZE 128

// 声明初始化串口、发送文件和接收文件的函数
int init_serial(const char *device, int baudrate);
int send_file(int fd, const char *filename);
int receive_file(int fd, const char *filename);

int main(int argc, char *argv[]) {
    if (argc != 6) {
        fprintf(stderr, "Usage: %s <device> <baudrate> <send|receive> <file>\n", argv[0]);
        return 1;
    }
    
    const char *device = argv[1];
    int baudrate = atoi(argv[2]);
    const char *mode = argv[3];
    const char *filename = argv[4];
    uint8_t   cmd_int = atoi(argv[5]);
    char      cmd     = argv[5][0];
    int res=0;
    printf("before init_serial\n");
    int fd = init_serial(device, baudrate);
    if (fd < 0) {
        return 1;
    }
    switch(cmd_int){
        case 1:
        case 2:
        case 3:
            if (write(fd, &cmd, 1) < 0) {
                perror("write");
                close(fd);
                return -1;
            }
            printf("send cmd ok\n");
            sleep(3);
            
            if (strcmp(mode, "send") == 0) {
                res = send_file(fd, filename);
            } else if (strcmp(mode, "receive") == 0) {
                res = receive_file(fd, filename);
            } else {
                fprintf(stderr, "Unknown mode: %s\n", mode);
                close(fd);
                return 1;
            }
            break;
        case 4:
        case 5:
            if (write(fd, &cmd, 1) < 0) {
                perror("write");
                close(fd);
                return -1;
            }
            break;
        default:
            printf("please input 1-5 at last");

    }


    close(fd);
    return res;
}

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

int send_file(int fd, const char *filename) {
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

int receive_file(int fd, const char *filename) {
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
