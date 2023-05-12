#ifndef _USER_BLE_H
#define _USER_BLE_H
#include "stdint.h"
#include "stdbool.h"

extern bool is_connect;
extern bool is_connect_ap;

void user_ble_init(void);
int save_remote_bound_add();
void comp_esp_ble_gatts_send_notify(uint8_t* temp, int len);
void comp_esp_ble_gattc_write_char(uint8_t* temp, int len);
//void spp_uart_init(void);
#endif