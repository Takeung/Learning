#include <stdio.h>
#include <stdint.h>
#include <string.h>

int charToHex(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1; // 非法字符
}

// 计算异或校验和的函数
uint8_t calculateChecksum(const char *data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 2; i < length - 4; i += 2) { // 跳过开始和结束标识
        checksum ^= charToHex(data[i]) * 16 + charToHex(data[i + 1]);
    }
    return checksum;
}

// 解析串口数据的函数
void parseSerialData(const char *data) {
    size_t length = strlen(data);
    if (length < 18) { // 确保数据长度足够
        printf("Received data is too short to be valid.\n");
        return;
    }

    // 跳过开始标识和结束标识，开始解析
    const char *payload = data + 2; // 跳过 `^^`
    size_t payloadLength = length - 4; // 减去 `^^` 和 `$$`

    // 计算实际数据的校验和
    uint8_t expectedChecksum = charToHex(payload[payloadLength - 2]) * 16 + charToHex(payload[payloadLength - 1]);
    uint8_t calculatedChecksum = calculateChecksum(data, length);

    if (calculatedChecksum != expectedChecksum) {
        printf("Checksum mismatch. Expected: 0x%02X, Calculated: 0x%02X\n", expectedChecksum, calculatedChecksum);
    } else {
        // 解析消息头
        int version = charToHex(payload[0]) * 16 + charToHex(payload[1]);
        int messageType = charToHex(payload[2]) * 16 + charToHex(payload[3]);
        int messageLength = charToHex(payload[4]) * 16 + charToHex(payload[5]);
        int operation = charToHex(payload[6]) * 16 + charToHex(payload[7]);
        printf("messageType is %d\n", messageType);
    }
}

int main()
{
	// char g_uart_in_data[] = "^^01060500000002$$";
	// uint8_t g_uart_in_data[] = "^^01070500000003$$";
	uint8_t g_uart_in_data[] = "^^0109050000000D$$";
	parseSerialData(&g_uart_in_data[0]);
	
	return 0;
}