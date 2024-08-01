#ifndef RFID_MODULE_H
#define RFID_MODULE_H

#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"

uint8_t* send_command(const uint8_t *cmd, size_t cmd_len);
uint8_t* set_power(int pwr, bool save);
float get_power();
uint8_t set_RF_mode(int mode, bool save);
uint8_t* read_start(uint16_t cycles);

uint8_t calculate_crc(const uint8_t *data, size_t length);

void print_response(const uint8_t *response, size_t length);

#endif // RFID_MODULE_H