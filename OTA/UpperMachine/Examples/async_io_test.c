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
#include <signal.h>
#include <termios.h>

typedef struct uart_cfg_t
{
    unsigned int baudrate;
    unsigned char databit;
    char parity;
    unsigned char stopbit;
} uart_cfg;

static struct termios old_cfg;
static int fd;
char *device = "/dev/ttysWK3";

static int uart_init(const char *device);
static int uart_config(const uart_cfg *cfg);
static void async_io_init(void);
static void io_handler(int sig, siginfo_t *info, void *context);

int main(int argc, char *argv[])
{
    uart_cfg cfg = {0};
    int ret;
    uart_init(device);
    cfg.baudrate = 115200;
    ret = uart_config(&cfg);
    if (ret)
    {
        tcsetattr(fd, TCSANOW, &old_cfg);
        close(fd);
        exit(EXIT_FAILURE);
    }
    async_io_init();
    while (1)
    {
        sleep(1);
    }
    tcsetattr(fd, TCSANOW, &old_cfg); // 恢复到之前的配置
    close(fd);
    exit(EXIT_SUCCESS);
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

/*UART配置*/
static int uart_config(const uart_cfg *cfg)
{
    struct termios new_cfg = {0};
    speed_t speed;
    cfmakeraw(&new_cfg);
    new_cfg.c_cflag |= CREAD;

    switch (cfg->baudrate)
    {
    case 9600:
        speed = B9600;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    default:
        speed = B115200;
        printf("default baud rate: 115200\n");
        break;
    }
    if (0 > cfsetspeed(&new_cfg, speed))
    {
        fprintf(stderr, "cfsetspeed error: %s\n)", strerror(errno));
    }
    new_cfg.c_cflag &= ~CSIZE;
    new_cfg.c_cflag |= CS8;
    new_cfg.c_cflag &= ~PARENB;
    new_cfg.c_iflag &= ~INPCK;
    new_cfg.c_cflag &= ~CSTOPB;

    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 0;

    if (0 > tcflush(fd, TCIOFLUSH))
    {
        fprintf(stderr, "tcflush error:%s\n", strerror(errno));
        return -1;
    }

    if (0 > tcsetattr(fd, TCSANOW, &new_cfg))
    {
        fprintf(stderr, "tcsetattr error: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/*UART初始化*/
static int uart_init(const char *device)
{
    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (0 > fd)
    {
        fprintf(stderr, "open error : %s: %s\n", device, strerror(errno));
        return -1;
    }

    if (0 > tcgetattr(fd, &old_cfg))
    {
        fprintf(stderr, "tcgetattr error : %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    return 0;
}

static void io_handler(int sig, siginfo_t *info, void *context)
{
    unsigned char buf[128] = {0};
    int ret;
    int n;

    if (SIGRTMIN != sig)
    {
        return;
    }

    if (POLL_IN == info->si_code)
    {
        ret = read(fd, buf, 128);
        for (n = 0; n < ret; n++)
        {
            printf("%c", buf[n]);
        }
    }
}
