#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

/* termios是posix规范中定义的标准接口，表示终端设备(包括虚拟终端、串口等)。*/
// #define NCC		18
// struct termios
// {
//     unsigned short c_iflag;  // 输入模式标志
//     unsigned short c_oflag;  // 输出模式标志
//     unsigned short c_cflag;  // 控制模式标志
//     unsigned short c_lflag;  // 本地模式标志
//     unsigned char c_line;    // 线路规程
//     unsigned char c_cc[NCC]; // 控制特性
//     speed_t c_ispeed;        // 输入速度
//     speed_t c_ospeed;        // 输出速度
// }

int main() {
    int serialFd = open("/dev/ttysWK3", O_RDWR);
    if (serialFd < 0) {
        perror("opening serial port error");
        return -1;
    }
 
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serialFd, &tty) != 0) {
        perror("Error configuring serial port");
        return -1;
    }
 
    // 配置串口参数
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
 
    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        perror("Error configuring serial port");
        return -1;
    }
 
    while(1) {
        // 发送数据
        char data[] = "Hello, world";
        write(serialFd , data, strlen(data));
        
        // 接收数据
        char buffer[255];
        int recvBytes= read(serialFd , buffer, sizeof(buffer));
        printf("Received: %.*s\n", recvBytes, buffer);
        sleep(1);
    }
   
    close(serialFd);
 
    return 0;
}
