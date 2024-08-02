#include <string.h>
#include <stdio.h>
#include <stdint.h>

void RearrangeUARTData(uint8_t *data, uint32_t size, uint32_t target_size) {
    if (size < target_size) {
        return; // Array is too small
    }

    // Check if the array starts with "^^" and ends with "$$"
    if (data[0] == '^' && data[1] == '^' && 
        data[target_size - 2] == '$' && data[target_size - 1] == '$') {
        return; // No need to rearrange
    }

    int start = -1, end = -1;

    // Find "^^" and "$$" in the array
    for (int i = 0; i < size - 1; i++) {
        if (data[i] == '^' && data[i + 1] == '^') {
            start = i;
        }
        if (data[i] == '$' && data[i + 1] == '$') {
            end = i + 1;
        }
    }

    if (data[0] == '^' && data[target_size - 1] == '^') {
        start = target_size - 1;
        end = target_size - 2;
    }

    if (data[0] == '$' && data[target_size - 1] == '$') {
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