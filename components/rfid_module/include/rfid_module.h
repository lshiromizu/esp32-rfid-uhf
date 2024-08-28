#ifndef RFID_MODULE_H
#define RFID_MODULE_H

#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"

#define MAX_EPC_LEN (62)

typedef struct {
    uint8_t EPC[MAX_EPC_LEN];   // Electronic Product Code
    float RSSI;                 // Received Signal Strength Indicator
    int count;                  // Number of times this tag has been read
} tag_t;

int send_command(const uint8_t *cmd, size_t cmd_len, uint8_t *response);
void set_power(int pwr, bool save);
float get_power();
void set_RF_mode(int mode, bool save);
int get_RF_mode();
int read_start(uint16_t cycles, tag_t *inventory);
void read_stop();
void software_reset();

uint8_t calculate_crc(const uint8_t *data, size_t length);
tag_t parse_tag_data(uint8_t data[]);

void print_cmd(const uint8_t *response, size_t length);

#endif // RFID_MODULE_H