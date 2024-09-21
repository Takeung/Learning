#include <stdio.h>
#include <time.h>

// 示例函数
void exampleFunction() {
    for (int i = 0; i < 100000000; i++); // 模拟耗时操作
}

int main() {
    clock_t start, end;
    double cpu_time_used;

    start = clock(); // 获取开始时间

    exampleFunction(); // 执行函数

    end = clock(); // 获取结束时间

    // 计算耗时，时钟周期除以每秒的时钟周期数（CLOCKS_PER_SEC）
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

    printf("函数执行时间: %f 秒\n", cpu_time_used);

    return 0;
}
