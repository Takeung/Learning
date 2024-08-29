#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

int main() {
    // 初始化随机数生成器
    srand(time(NULL));

    // 随机生成一个0到99之间的浮点数
    // float input = (float)(rand() % 100);
    float input = 13.576;
    
    // 计算整数部分和小数部分
    int integer_part = (int)input;
    int decimal_part = (int)((input - integer_part) * 100 + 0.5);

    // 分解整数部分
    int integer_high = integer_part / 10;  // 十位数
    int integer_low = integer_part % 10;   // 个位数

    // 四舍五入小数部分
    int decimal_high = decimal_part / 10;  // 小数点后第一位
    int decimal_low = decimal_part % 10;   // 小数点后第二位

    // 计算字节值
    uint8_t high_byte = (integer_high << 4) | integer_low;
    uint8_t low_byte = (decimal_high << 4) | decimal_low;

    // 输出结果
    printf("Input number: %.4f\n", input);
    printf("High byte: 0x%02X\n", high_byte);
    printf("Low byte: 0x%02X\n", low_byte);
    printf("Combined: 0x%02X 0x%02X\n", high_byte, low_byte);

    return 0;
}
