#include <stdint.h>
#include <stdio.h>

void processBusVoltage(uint16_t raw_data) {
    // 判断是否为负数并获取绝对值
    uint16_t signed_data = raw_data;
    if (raw_data & 0x8000) { // 最高位是符号位，判断是否为负数
        signed_data = (int16_t)((~signed_data + 1) & 0xFFFF); // 获取绝对值
    } else {
        signed_data = raw_data; // 正数直接使用原始值
    }
    printf("signed_data: %u\n", signed_data);

    // 去掉无效的低三位
    uint16_t processed_data = signed_data >> 3;

    // 输出处理后的结果
    printf("Processed Bus Voltage: %u\n", processed_data);
}

int main() {
    uint16_t raw_data = 0xA1B3; // 示例数据
    processBusVoltage(raw_data);
    raw_data = 0x7FF8; // 示例数据
    processBusVoltage(raw_data);

    return 0;
}
