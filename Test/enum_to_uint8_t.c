#include <stdint.h>
#include <stdio.h>

typedef enum {
    VOLTAGE_NORMAL = (uint8_t)0,
    VOLTAGE_OUT_OF_NORMAL = (uint8_t)1,
    VOLTAGE_ALERT = (uint8_t)2
} VoltageStatus;

void heart_beat_process(uint8_t v_status)
{
    switch (v_status) {
        case 0:
            printf("NO_FAULT_WAVE()\n");
            break;
        case 1:
            printf("MINOR_FAULT_WAVE()\n");
            break;
        case 2:
        case 3:
            printf("MAJOR_FAULT_WAVE()\n");
            break;
        default:
            break;
    }
}

int main() {
    VoltageStatus adc_voltage_status = VOLTAGE_NORMAL;
    VoltageStatus ina3221_voltage_status = VOLTAGE_NORMAL;

    adc_voltage_status |= VOLTAGE_OUT_OF_NORMAL;
    printf("adc_voltage_status is: %u\n", adc_voltage_status);
    adc_voltage_status |= VOLTAGE_ALERT;
    printf("adc_voltage_status is: %u\n", adc_voltage_status);
    adc_voltage_status |= VOLTAGE_NORMAL;
    printf("adc_voltage_status is: %u\n", adc_voltage_status);
    ina3221_voltage_status = VOLTAGE_ALERT;

    heart_beat_process(adc_voltage_status | ina3221_voltage_status);

    char status_buffer[512];
    char *ptr_status = status_buffer;
    ptr_status += sprintf(ptr_status, "78963\r\n");
    printf("str_len(status_buffer) is: %lu\r\n", strlen(status_buffer));

    return 0;
}
