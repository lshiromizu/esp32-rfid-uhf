#ifndef RFID_MODULE_H
#define RFID_MODULE_H

#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"
#include <time.h>

#define MAX_EPC_LEN (62)

typedef struct {
    uint8_t EPC[MAX_EPC_LEN];   // Electronic Product Code
    size_t epc_lenght;          // EPC length (variable depending on tags)
    float RSSI;                 // Received Signal Strength Indicator
    int count;                  // Number of times this tag has been read
} tag_t;

int send_command(const uint8_t *cmd, size_t cmd_len, uint8_t *response, time_t timeout);
void set_power(int pwr, bool save);
float get_power();
void set_RF_mode(int mode, bool save);
int get_RF_mode();
tag_t read_tag(uint16_t timeout);
void read_start();
void read_stop();
int get_inventory(tag_t *inventory, size_t inv_size);
int stop_get_inventory(tag_t *inventory, size_t inv_size);
void default_config();
tag_t* tag_select(tag_t* inventory, size_t tags_count);

uint8_t calculate_crc(const uint8_t *data, size_t length);
tag_t parse_tag_data(uint8_t data[]);

void print_cmd(const uint8_t *response, size_t length);

#endif // RFID_MODULE_H