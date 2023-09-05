#ifndef _ESP32_I2C_H
#define _ESP32_I2C_H

int imu_i2c_init(void);
int I2C_WriteData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);
int I2C_ReadData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);
#endif
