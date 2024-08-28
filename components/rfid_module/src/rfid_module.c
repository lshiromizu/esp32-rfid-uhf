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
#define BUF_SIZE (128)
#define TIMEOUT 10


int send_command(const uint8_t *cmd, size_t cmd_len, uint8_t *response) {
    uint8_t tx_buffer[BUF_SIZE];
    size_t full_len = cmd_len + 7; // add SOF(2) + len(2) + CRC(1) + EOF(2)

    // Build the command frame
    tx_buffer[0] = 0xA5;
    tx_buffer[1] = 0x5A;
    tx_buffer[2] = (full_len >> 8) & 0xFF;
    tx_buffer[3] = (full_len) & 0xFF;
    memcpy(&tx_buffer[4], cmd, cmd_len);
    uint8_t crc_data[cmd_len + 2];
    crc_data[0] = tx_buffer[2];
    crc_data[1] = tx_buffer[3];
    memcpy(&crc_data[2], cmd, cmd_len);
    tx_buffer[4 + cmd_len] = calculate_crc(crc_data, cmd_len + 2); // Calculate CRC of len + cmd + data
    tx_buffer[5 + cmd_len] = 0x0D;
    tx_buffer[6 + cmd_len] = 0x0A;

    print_cmd(tx_buffer, full_len);
    int tx_bytes = uart_write_bytes(UART_NUM, tx_buffer, full_len);

    if (tx_bytes != full_len) {
        ESP_LOGE("UART", "Failed to send complete message");
        return -1;
    }

    // Receive the response
    int rx_bytes = uart_read_bytes(UART_NUM, response, BUF_SIZE, pdMS_TO_TICKS(100));
    if (rx_bytes < 0) {
        ESP_LOGE("UART", "Failed to read response");
        return -1;
    }
    uart_flush(UART_NUM);

    return rx_bytes;
}

void set_power(int pwr, bool save) {
    uint8_t cmd[7];
    cmd[0] = 0x10; // Command for setting reading output power
    cmd[1] = save ? 0x20 : 0x00; //save option
    cmd[2] = 0x01;

    uint16_t hex = (uint16_t)(pwr * 100);
    cmd[3] = (hex >> 8) & 0xFF; // High byte of read power
    cmd[4] = hex & 0xFF; // Low byte of read power
    cmd[5] = cmd[3]; // High byte of write power, same as read power
    cmd[6] = cmd[4]; // Low byte of write power, same as read power

    uint8_t response[BUF_SIZE];
    int rx_bytes = send_command(cmd, sizeof(cmd), response);
    if (rx_bytes == 0) {
        ESP_LOGE("UART", "No response received or error occurred");
    }

    if (response[5] == 0x01){
        printf("Power set. \r\n");
    }else{
        printf("Error. \r\n");
    }

}

float get_power() {
    uint8_t cmd[1] = {0x12};
    uint8_t response[BUF_SIZE];

    int rx_bytes = send_command(cmd, sizeof(cmd), response);
    if (rx_bytes == 0) {
        ESP_LOGE("UART", "No response received or error occurred");
        return -1;
    }
    // Debug: Print the received bytes
    //print_cmd(response, rx_bytes);

    // Extract the power value from the 8th and 9th bytes
    int16_t pwr_hex = (response[7] << 8) | response[8];
    float pwr = (float)pwr_hex / 100.0;
    return pwr;
}

void set_RF_mode(int mode, bool save){
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
    }
    
    uint8_t response[BUF_SIZE];
    int rx_bytes = send_command(cmd, sizeof(cmd), response);
    if (rx_bytes == 0) {
        ESP_LOGE("UART", "No response received or error occurred");
    }

    if (response[5] == 0x01){
        printf("RF mode set. \r\n");
    }else{
        printf("Error. \r\n");
    }
}

int get_RF_mode(){
    uint8_t cmd[3];
    cmd[0] = 0x54;
    cmd[1] = 0x00;
    cmd[2] = 0x00;

    uint8_t response[BUF_SIZE];
    int rx_bytes = send_command(cmd, sizeof(cmd), response);
    if (rx_bytes == 0) {
        ESP_LOGE("UART", "No response received or error occurred");
        return -1;
    }
    // Debug: Print the received bytes
    //print_cmd(response, rx_bytes);

    return (int)response[7];
}

int read_start(uint16_t cycles, tag_t *inventory) {
    uint8_t cmd[3];
    cmd[0] = 0x82; // Command for reading
    cmd[1] = (cycles >> 8) & 0xFF;
    cmd[2] = cycles & 0xFF;

    uint8_t response[cycles][BUF_SIZE];
    send_command(cmd, sizeof(cmd), response[0]);

    for (size_t i = 0; i < (cycles - 1); i++)
    {
        uart_read_bytes(UART_NUM, response[i+1], BUF_SIZE, pdMS_TO_TICKS(10));
    }
    read_stop();

    int tags_count = 0;  // inventory size
    for (size_t i = 0; i < cycles; i++) {
        tag_t tag = parse_tag_data(response[i]);
        int j = 0;
        while (j <= tags_count){
            if (memcmp(inventory[j].EPC, tag.EPC, sizeof(tag.EPC)) == 0) {
                if (tag.RSSI > inventory[j].RSSI)
                {
                    inventory[j].RSSI = tag.RSSI;
                }
                inventory[j].count++;
                break;
            }
            if (j == tags_count){
                inventory[j] = tag;
                inventory[j].count = 1;
                tags_count++;
                break;
            }
            j++;
        }
    }
    return tags_count;
}

void read_stop(){
    uint8_t cmd[1] = {0x8C};
    uint8_t response[BUF_SIZE];

    int rx_bytes = send_command(cmd, sizeof(cmd), response);
    if (rx_bytes == 0) {
        ESP_LOGE("UART", "No response received or error occurred");
    }
    // Debug: Print the received bytes
    //print_cmd(response, rx_bytes);
    if (response[5] == 0x01){
        printf("Done reading. \r\n");
    }else{
        printf("Error. \r\n");
    }
}

void software_reset(){
    uint8_t cmd[1] = {0x2A};
    uint8_t response[BUF_SIZE];

    int rx_bytes = send_command(cmd, sizeof(cmd), response);
    if (rx_bytes == 0) {
        ESP_LOGE("UART", "No response received or error occurred");
    }
    // Debug: Print the received bytes
    //print_cmd(response, rx_bytes);
}

uint8_t calculate_crc(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

tag_t parse_tag_data(uint8_t data[]) {
    tag_t parsed_data;
    uint16_t pc = (data[5] << 8) | data[6];
    size_t epc_length = ((pc >> 11) & 0x1F) * 2;
    for (size_t i = 0; i < epc_length; i++)
    {
        parsed_data.EPC[i]=data[i+7];
    }
    int16_t rssi_hex = (data[epc_length+7] << 8) | data[epc_length+8];
    parsed_data.RSSI = (float)rssi_hex / 10.0;

    return parsed_data;
}

void print_cmd(const uint8_t *response, size_t length) {
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", response[i]);
    }
    printf("\r\n");
}