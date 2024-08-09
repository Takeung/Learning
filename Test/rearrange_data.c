#include <string.h>
#include <stdio.h>
#include <stdint.h>

#define START_FLAG     0x5E  // '^'字符的ASCII码
#define END_FLAG       0x24  // '$'字符的ASCII码

void RearrangeUARTData(uint8_t *data, uint32_t size, uint32_t target_size) {
    if (size < target_size) {
        return; // Array is too small
    }

    // Check if the array starts with "^^" and ends with "$$"
    if (data[0] == START_FLAG && data[1] == START_FLAG && 
        data[target_size - 2] == END_FLAG && data[target_size - 1] == END_FLAG) {
        return; // No need to rearrange
    }

    int start = -1, end = -1;

    // Find "^^" and "$$" in the array
    for (int i = 0; i < size - 1; i++) {
        if (data[i] == START_FLAG && data[i + 1] == START_FLAG) {
            start = i;
        }
        if (data[i] == END_FLAG && data[i + 1] == END_FLAG) {
            end = i + 1;
        }
    }

    if (data[0] == START_FLAG && data[target_size - 1] == START_FLAG) {
        start = target_size - 1;
        end = target_size - 2;
    }

    if (data[0] == END_FLAG && data[target_size - 1] == END_FLAG) {
        start = 1;
        end = 0;
    }
    
    // Rearrange the array if "^^" and "$$" are found
    if (start != -1 && end != -1) {
        uint8_t temp[target_size];
        int tempIndex = 0;

        // Copy the sequence starting from "^^" to the beginning of the temp array
        for (int i = start; i < size; i++) {
            temp[tempIndex++] = data[i];
        }
        tempIndex = target_size - 1;
        // Copy the sequence ending with "$$" in reverse order to the end of the temp array
        for (int i = end; i >= 0; i--) {
            temp[tempIndex--] = data[i];
        }

        // Copy the rearranged temp array back to the original array
        memcpy(data, temp, target_size);
    }
}

void PrintArray(uint8_t *data, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        printf("%c", data[i]);
    }
    printf("\n");
}

int main() {
    uint8_t g_all_data[36] = "^^01060500000002$$^^01060500000002$$";
    for (int i = 0; i < 18; i++) {
        uint8_t g_uart8_in_data[18];
        memcpy(g_uart8_in_data, g_all_data + i, 18);
        printf("Original data: ");
        PrintArray(g_uart8_in_data, 18);

        RearrangeUARTData(g_uart8_in_data, 18, 18);

        printf("Rearranged data: ");
        PrintArray(g_uart8_in_data, 18);
    }

    return 0;
}