#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "rfid_module.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define TIMEOUT 10


uint8_t* send_command(const uint8_t *cmd, size_t cmd_len) {
    static uint8_t tx_buffer[BUF_SIZE];
    static uint8_t rx_buffer[BUF_SIZE];

    // Build the command frame
    tx_buffer[0] = 0xA5;
    tx_buffer[1] = 0x5A;
    tx_buffer[2] = ((cmd_len + 7) >> 8) & 0xFF;
    tx_buffer[3] = (cmd_len + 7) & 0xFF;
    memcpy(&tx_buffer[4], cmd, cmd_len);
    uint8_t crc_data[cmd_len + 2];
    crc_data[0] = tx_buffer[2]; // Length MSB
    crc_data[1] = tx_buffer[3]; // Length LSB
    memcpy(&crc_data[2], cmd, cmd_len);
    tx_buffer[4 + cmd_len] = calculate_crc(crc_data, cmd_len + 2); // Calculate CRC
    tx_buffer[5 + cmd_len] = 0x0D;
    tx_buffer[6 + cmd_len] = 0x0A;

    print_response(tx_buffer, cmd_len + 7);
    uart_write_bytes(UART_NUM, tx_buffer, cmd_len + 7);

    // Receive the response
    int rx_len = uart_read_bytes(UART_NUM, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    if (rx_len < 0) {
        ESP_LOGE("RFID", "UART read error");
        return NULL;
    }

    // Allocate memory for the response
    uint8_t *response = (uint8_t *)malloc(rx_len);
    if (response == NULL) {
        ESP_LOGE("RFID", "Memory allocation failed");
        return NULL;
    }
    memcpy(response, rx_buffer, rx_len);

    print_response(response, sizeof(response));

    return response;
}

uint8_t* set_power(int pwr, bool save) {
    uint8_t cmd[7];
    cmd[0] = 0x10; // Command for setting reading output power
    cmd[1] = save ? 0x20 : 0x00;
    cmd[2] = 0x01;

    uint16_t hex = (uint16_t)(pwr * 100);
    cmd[3] = (hex >> 8) & 0xFF; // High byte of read power
    cmd[4] = hex & 0xFF; // Low byte of read power
    cmd[5] = cmd[3]; // High byte of write power
    cmd[6] = cmd[4]; // Low byte of write power

    return send_command(cmd, sizeof(cmd));
}

float get_power() {
    uint8_t cmd[1] = {0x12};

    uint8_t *response = send_command(cmd, sizeof(cmd));
    if (response == NULL) {
        ESP_LOGE("RFID", "Failed to get power");
        return -1;
    }
    print_response(response, 10);  // assuming response is 10 bytes for example

    // Extract the power value from the 7th and 8th bytes
    int16_t pwr_hex = (response[6] << 8) | response[7];
    free(response);

    float pwr = (float)pwr_hex / 100.0;
    return pwr;
}

uint8_t set_RF_mode(int mode, bool save){
    /*    Set device RF link mode.

        Parameters: 
        mode (int): 0 -> DSB_ASK /FM0/ 40 KHz
                    1 -> PR _ASK /Miller4/ 250KHz
                    2 -> PR _ASK /Miller4/ 300KHz
                    3 -> DSB_ASK /FM0/ 400KHz
        save (bool): If True, saves the configuration to NVM. Defaults to False.

        Returns:
        response: The device response, bytearray.
    */
    uint8_t cmd[4];
    
    cmd[0] = 0x52;
    cmd[1] = 0x00;
    cmd[2] = save ? 0x01 : 0x00;

    switch (mode)
    {
    case 0:
        cmd[3] = 0x00;
        break;
    case 1:
        cmd[3] = 0x01;
        break;
    case 2:
        cmd[3] = 0x02;
        break;
    case 3:
        cmd[3] = 0x03;
        break;
    default:
        ESP_LOGE("RFID", "Invalid mode");
        return NULL;
    }
    
    return send_command(cmd, sizeof(cmd));
}


uint8_t* read_start(uint16_t cycles) {
    uint8_t cmd[3];
    cmd[0] = 0x82; // Command for reading
    cmd[1] = (cycles >> 8) & 0xFF; // High byte of cycles
    cmd[2] = cycles & 0xFF; // Low byte of cycles

    // Send the start read command
    uint8_t *initial_response = send_command(cmd, sizeof(cmd));
    if (initial_response == NULL) {
        ESP_LOGE("RFID", "Failed to send initial read command");
        return NULL;
    }
    free(initial_response);

    // Continuously read responses
    static uint8_t rx_buffer[BUF_SIZE];
    size_t inventory_size = 0;
    uint8_t *inventory = NULL;

    while (1) {
        int rx_len = uart_read_bytes(UART_NUM, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rx_len <= 0) {
            ESP_LOGE("RFID", "UART read error");
            break;
        }

        // Reallocate memory to store new tag data
        inventory = (uint8_t *)realloc(inventory, inventory_size + rx_len);
        if (inventory == NULL) {
            ESP_LOGE("RFID", "Memory allocation failed");
            return NULL;
        }

        // Copy the new tag data to the inventory
        memcpy(inventory + inventory_size, rx_buffer, rx_len);
        inventory_size += rx_len;
    }

    return inventory;
}

uint8_t calculate_crc(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

void print_response(const uint8_t *response, size_t length) {
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", response[i]);
    }
    printf("\r\n");
}