#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "imu"
#define RETRY_COUNT 3

#define I2C_MASTER_SCL_IO 33          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 32          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0              /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ     400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
/**
 * @fn          imu_init
 * @brief       初始化
 * @param       none
 * @return      非0错误
 * @note
 */
esp_err_t imu_i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret;
    ret = i2c_param_config(i2c_master_port, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "config fail %d", ret);
        return ESP_FAIL;
    }
    ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "init fail %d", ret);
        return ESP_FAIL;
    }
    return ESP_OK;
}

int I2C_WriteData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    assert (cmd != NULL);

    ret = i2c_master_start(cmd);
    if(ESP_OK != ret)
    {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, (slaveAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    if(ESP_OK != ret)
    {
        goto end;
    }


    ret = i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    if(ESP_OK != ret)
    {
        goto end;
    }   


    ret = i2c_master_write(cmd, pData, dataLen, ACK_CHECK_EN);
    if(ESP_OK != ret)
    {
        goto end;
    }

    ret = i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret;

end:
    printf("iic write err /n");
    i2c_cmd_link_delete(cmd);
    return ret;
}

int I2C_ReadData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    assert (cmd != NULL);

    ret = i2c_master_start(cmd);
    if(ESP_OK != ret)
    {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, (slaveAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    if(ESP_OK != ret)
    {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    if(ESP_OK != ret)
    {
        goto end;
    }

    ret = i2c_master_start(cmd);
    if(ESP_OK != ret)
    {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, (slaveAddr << 1) | READ_BIT, ACK_CHECK_EN);
    if(ESP_OK != ret)
    {
        goto end;
    }

#if 0
    if(dataLen > 1)
    {
        ret = i2c_master_read(cmd, pData, dataLen-1, ACK_VAL);
        if(ESP_OK != ret)
        {
            goto end;
        }
    }
    ret = i2c_master_read(cmd, pData+dataLen-1, 1, NACK_VAL);
    if(ESP_OK != ret)
    {
        goto end;
    }
#else
    ret = i2c_master_read(cmd, pData, dataLen, I2C_MASTER_LAST_NACK);
    if(ESP_OK != ret)
    {
        goto end;
    }

#endif

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret;

end:
    printf("iic read err /n");
    i2c_cmd_link_delete(cmd);
    return ret;
}


