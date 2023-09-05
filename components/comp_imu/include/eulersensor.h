#ifndef EULERSENSOR_H
#define EULERSENSOR_H
#include <stdint.h>
#include <string.h>

void imu_thread_run(void);
void is_en_check_gyr_slot(void);
void is_en_check_amg_slot(void);
void imu_get_atti(float* roll, float*pitch, float*yaw);
void imu_regist_uart_handle(int (*p_fun)(int uart_num, const void *src, size_t size));
#endif
