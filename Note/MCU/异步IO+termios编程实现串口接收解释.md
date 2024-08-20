# 异步I/O+termios编程实现串口接收解释

termios是面向所有终端设备的。
termios 结构体：

```c
   tcflag_t c_iflag;      /* input modes */
   tcflag_t c_oflag;      /* output modes */
   tcflag_t c_cflag;      /* control modes */
   tcflag_t c_lflag;      /* local modes */
   cc_t     c_cc[NCCS];   /* special characters */
```

### 终端的三种模式

**规范模式（命令行的形式）**
所有输入基于行进行处理。在用户输入一个行结束符（回车符、EOF等）之前，系统调用`read()`函数读不到用户输入的任何字符。其次，除了EOF之外的行结束符与普通字符一样会被`read()`函数读取到缓冲区中。一次调用`read()`只能读取一行数据。
**非规范模式**
所有输入是即时有效的，不需要用户另外输入行结束符。
**原始模式**
是一种特殊的非规范模式，所有的输入数据以字节为单位被处理。即有一个字节输入时，触发输入有效。

但是串口并不仅仅只扮演着人机交互的角色（数据以字符的形式传输、也就数说传输的数据其实字符对应的 ASCII 编码值）；串口本就是一种数据串行传输接口，通过串口可以与其他设备或传感器进行数据传输、通信，譬如很多 sensor 就使用了串口方式与主机端进行数据交互。那么在这种情况下，我们就得使用原始模式，意味着通过串口传输的数据不应进行任何特殊处理、不应将其解析成 ASCII 字符。

### 终端控制API函数

```c
tcgetattr      取属性(termios结构)
tcsetattr      设置属性(termios结构)
cfgetispeed    得到输入速度
cfgetospeed    得到输出速度
cfsetispeed    设置输入速度
cfsetospeed    设置输出速度
tcdrain        等待所有输出都被传输
tcflow         挂起传输或接收
tcflush        刷清未决输入和/或输出
tcsendbreak    送BREAK字符
tcgetpgrp      得到前台进程组ID
tcsetpgrp      设置前台进程组ID
cfmakeraw    将终端设置成原始模式
cfsetspeed   设置输入输出速度
```

需要注意的地方：

```c
fd = open(device,O_RDWR|O_NOCTTY|O_NDELAY);
```

1. `O_NONBLOCK`/如果`pathname`指的是一个FIFO、一个块特殊文件或一个字符特殊文件，则此选择项为此文件的本次打开操作和后续的I/O操作设置非阻塞方式。
2. `O_NOCTTY` 如果`pathname`指的是终端设备，则不将此设备分配作为此进程的控制终端。（个人的理解是只有read和write才能对指定此终端设备进行通信）。
3. `O_NONBLOCK`和`O_NDELAY`几乎相同，它们的差别在于设立`O_NDELAY`会使I/O函式马上回传0，但是又衍生出一个问题，因为读取到档案结尾时所回传的也是0，这样无法得知是哪中情况；因此，`O_NONBLOCK`就产生出来，它在读取不到数据时会回传-1，并且设置errno为`EAGAIN`。
4. fd是int类型，device是char*指针，eg：“`/dev/ttyUSB0`”。

```c
tcgetattr(fd, &old_cfg)；
```

获取终端设备的参数，保存至old_cfg中，old_cfg是自己设定的全局变量，类型是struct termios。old_cfg的作用，退出主循环后，还要将终端设备的工作模式恢复成规范模式（命令行模式解析成ASCII码）。

```c
tcsetattr(fd, TCSANOW, &old_cfg);
close(fd);
```

中间的参数是配置立即生效

```C
cfmakeraw(&new_cfg);
```

将终端设备设置为原始模式

```C
new_cfg.c_cflag |= CREAD;	//接收模式
cfsetspeed(&new_cfg,speed)
```

设置输入输出baud率，speed是speed_t类型的

```C
new_cfg.c_cflag &= ~CSIZE; //清空数据位控制字
new_cfg.c_cflag |= CS8;		//数据位八位
new_cfg.c_cflag &= ~PARENB;	//配置为无校验
new_cfg.c_iflag &= ~INPCK;		//配置为无校验
new_cfg.c_cflag &= ~CSTOPB;	//一个停止位，如果是或上就是两个停止位
new_cfg.c_cc[VTIME] = 0;
new_cfg.c_cc[VMIN] = 0;
```

在对接收字符和等待时间没有特别要求的情况下，可以将 MIN 和 TIME 设置为 0，这样则在任何情况下 read()调用都会立即返回，此时对串口的 read 操作会设置为非阻塞方式

### 异步I/O配置使用

类似于使用中断

```c
int flag;
flag = fcntl(fd, F_GETFL);             //先获取原来的 flag
flag |= O_ASYNC;                         //将 O_ASYNC 标志添加到  flag 
fcntl(fd, F_SETFL, flag);               //重新设置  flag
```

```c
fcntl(fd, F_SETOWN, getpid());  
```

为文件描述符设置异步 I/O 事件的接收进程，也就是设置异步 I/O 的所有者。

```c
fcntl(fd, F_SETSIG, SIGRTMIN); 
```

`SIGRTMIN`编号是34，小于34的都是不可靠信号。
`SIGRTMAX`编号是64号。
指定实时信号SIGRTMIN作为异步I/O通知信号。 `SIGRTMIN->(SIG-REAL-TIME-MINIMUM)`是实时信号
注意： `F_SETSIG`要使用`#define _GNU_SOURCE` 宏定义

```c
struct sigaction act;
act.sa_sigaction = io_handler; //sa_sigaction是个函数指针，指向相应的处理函数
act.sa_flags = SA_SIGINFO; 
sigemptyset(&act.sa_mask); 
sigaction(SIGRTMIN, &act, NULL);
```

下面是例程代码

```c
#define _GNU_SOURCE //在源文件开头定义_GNU_SOURCE宏
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
}
uart_cfg;

static struct termios old_cfg;
static int fd;
char *device = "/dev/ttyUSB0";

static int uart_init(const char *device);
static int uart_config(const uart_cfg *cfg);
static void async_io_init(void);
static void io_handler(int sig,siginfo_t *info, void *context);

int main(int argc, char *argv[])
{
    uart_cfg cfg = {0};
    int ret;
    uart_init(device);
    cfg.baudrate = 115200;
    ret = uart_config(&cfg);
    if(ret)
    {
        tcsetattr(fd, TCSANOW, &old_cfg);
        close(fd);
        exit(EXIT_FAILURE);
    }
    async_io_init();
    while(1)
    {
        sleep(1);
    }
    tcsetattr(fd, TCSANOW, &old_cfg); //恢复到之前的配置 
    close(fd); 
    exit(EXIT_SUCCESS);

}


/*异步IO初始化函数*/
static void async_io_init(void)
{
    struct sigaction sig;
    int flag;

/*使能异步I/O*/            /**/
flag = fcntl(fd,F_GETFL); /*使能串口的异步I/O功能*/
flag |= O_ASYNC;
fcntl(fd,F_SETFL,flag);
/*设置异步I/O的所有者*/
fcntl(fd,F_SETOWN,getpid());
/*为实时信号SIGRTMIN作为异步I/O信号*/
fcntl(fd,F_SETSIG,SIGRTMIN);

sig.sa_sigaction = io_handler;
sig.sa_flags = SA_SIGINFO;
sigisemptyset(&sig.sa_mask);
sigaction(SIGRTMIN,&sig,NULL);

}

/*UART配置*/
static int uart_config(const uart_cfg *cfg)
{
    struct termios new_cfg = {0};
    speed_t speed;
    cfmakeraw(&new_cfg);
    new_cfg.c_cflag |= CREAD;

switch(cfg->baudrate)
{
    case 9600: speed = B9600; 
    break;
    case 38400: speed = B38400;
    break;
    case 57600: speed = B57600;
    break;
    case 115200: speed = B115200;
    break;
    default: 
        speed = B115200;
        printf("default baud rate: 115200\n");
        break;
}

if(0 > cfsetspeed(&new_cfg,speed))
{
    fprintf(stderr,"cfsetspeed error: %s\n)",strerror(errno));
}
    new_cfg.c_cflag &= ~CSIZE;
    new_cfg.c_cflag |= CS8;
    new_cfg.c_cflag &= ~PARENB;
    new_cfg.c_iflag &= ~INPCK;
    new_cfg.c_cflag &= ~CSTOPB;

new_cfg.c_cc[VTIME] = 0;
new_cfg.c_cc[VMIN] = 0;

if(0 > tcflush(fd,TCIOFLUSH))
{
    fprintf(stderr,"tcflush error:%s\n",strerror(errno));
    return -1;
}

if(0 > tcsetattr(fd,TCSANOW,&new_cfg))
{
    fprintf(stderr,"tcsetattr error: %s\n",strerror(errno));
    return -1;
}
return 0;

}

/*UART初始化*/
static int uart_init(const char *device)
{
    fd = open(device,O_RDWR|O_NOCTTY|O_NDELAY);
    if(0 > fd)
    {
        fprintf(stderr, "open error : %s: %s\n",device,strerror(errno));
        return -1;        
    }

if(0 > tcgetattr(fd, &old_cfg))
{
    fprintf(stderr, "tcgetattr error : %s\n",strerror(errno));
    close(fd);
    return -1;
}
return 0;

}

static void io_handler(int sig,siginfo_t *info, void *context)
{
    unsigned char buf[128] = {0};
    int ret;
    int n;

if(SIGRTMIN != sig)
{
    return;
}

if(POLL_IN == info->si_code)
{
    ret = read(fd,buf,128);
    for(n = 0;n < ret ;n++)
    {

//            printf("0x%hhx ",buf[n]);
            printf("%hhc",buf[n]);
        }
    }
}
```

注意：

```c
ret = read(fd,buf,128);
```

ret返回的值是读取到接收缓冲区中的数据长度。

```C
sig.sa_flags = SA_SIGINFO;
```

`SA_SIGINFO`控制字代表`sa_sigaction`成员有效，`sa_handler`成员无效。
`sa_sigaction`和`sa_handler`都是函数指针。
区别是`sa_sigaction`的参数信息更多。