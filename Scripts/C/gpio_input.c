#include<sys/types.h>
#include<sys/stat.h>
#include<stdio.h>
#include<poll.h>
#include<fcntl.h>
#include<time.h>
#include<string.h>
#include<sys/time.h>
#include<unistd.h>
#include<math.h>

int main(void)
{
    unsigned char data[24];
    struct pollfd fds[1];
    struct timeval tv;
    memset(data, 0, 24);
    int fd = open("/dev/mcu-gpio4", O_RDWR);
    if (fd < 0) {
        printf("can not open file\n");
        return 0;
    }
    fds[0].fd = fd;
    fds[0].events = POLLIN;
    gettimeofday(&tv, NULL);
    long last_s = tv.tv_sec;
    long last_us = tv.tv_usec;
    while (1) {
        if (poll(fds, 1, 10000) == 0) {
            printf("timeout\n");
        } else {
            gettimeofday(&tv, NULL);
            // printf("tv.tv_sec: %lu, tv.tv_usec: %lu\r\n", tv.tv_sec, tv.tv_usec);
            // printf("last_s: %lu, last_us: %lu\r\n", last_s, last_us);
            // if ((tv.tv_sec - last_s == 1) && (tv.tv_usec - last_us < 100 || last_us - tv.tv_usec < 100)) {
            //     last_s = tv.tv_sec;
            //     last_us = tv.tv_usec;
            //     printf("\r\n");
            // }
            read(fd, data, 1);
            printf("%lu.%lu, %c\r\n", tv.tv_sec, tv.tv_usec, data[0]);
            // printf((data[0] == '0') ? "_" : "^");
        }
    }

    close(fd);
    return 0;
}