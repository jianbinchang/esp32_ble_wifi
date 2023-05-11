
#include "user_sdio.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"

#include "driver/sdio_slave.h"

#include "soc/slc_struct.h"
#include "soc/slc_reg.h"
#include "soc/host_struct.h"
#include "soc/soc.h"

#include "nvs_flash.h"

#define ESP_AT_SDIO_BUFFER_SIZE      2048
#define ESP_AT_SDIO_BUFFER_NUM       10
#define ESP_AT_SDIO_QUEUE_SIZE       10

extern QueueHandle_t Tcp_To_Sdio_Queue_Handle;
extern QueueHandle_t Sdio_to_Tcp__Queue_Handle;

static const char* TAG = "esp32_sdio";

uint8_t WORD_ALIGNED_ATTR pbuf[ESP_AT_SDIO_BUFFER_NUM][ESP_AT_SDIO_BUFFER_SIZE];

// send them to SDIO
static int32_t esp32_sdio_write_data(uint8_t* data, int32_t len)
{
    esp_err_t ret = ESP_OK;
    int32_t length = len;
    uint8_t* sendbuf = NULL;
    if (len < 0 || data == NULL) {
        ESP_LOGE(TAG , "Write data error, len:%d", len);
        return -1;
    }

    sendbuf = heap_caps_malloc(len, MALLOC_CAP_DMA);
    if (sendbuf == NULL) {
        ESP_LOGE(TAG , "Malloc send buffer fail!");
        return 0;
    }

    memcpy(sendbuf, data, length);

    ret = sdio_slave_transmit(sendbuf, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG , "sdio slave transmit error, ret : 0x%x\r\n", ret);
        length = 0;
    }

    free(sendbuf);
    return length;
}


static void esp32_sdio_slave_init()
{
    sdio_slave_config_t config = {
        .sending_mode       = SDIO_SLAVE_SEND_STREAM,
        .send_queue_size    = ESP_AT_SDIO_QUEUE_SIZE,
        .recv_buffer_size   = ESP_AT_SDIO_BUFFER_SIZE,
    };
    sdio_slave_buf_handle_t handle;

    esp_err_t ret = sdio_slave_initialize(&config);
    assert(ret == ESP_OK);

    for (int loop = 0; loop < ESP_AT_SDIO_BUFFER_NUM; loop++) {
        handle = sdio_slave_recv_register_buf(pbuf[loop]);
        assert(handle != NULL);

        ret = sdio_slave_recv_load_buf(handle);
        assert(ret == ESP_OK);
    }

    sdio_slave_set_host_intena(SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET | SDIO_SLAVE_HOSTINT_BIT0);

    sdio_slave_start();

    ESP_LOGI(TAG, "slave ready1");
}

static void at_sdio_recv_task(void* pvParameters)
{
    sdio_slave_buf_handle_t handle;
    size_t length = 0;
    uint8_t* ptr = NULL;

    for (;;) {
        // receive data from SDIO host
        esp_err_t ret = sdio_slave_recv(&handle, &ptr, &length, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Recv error,ret:%x", ret);
            continue;
        }

        printf("SDIO slave receive(%d)\n", length);

        ESP_LOG_BUFFER_HEXDUMP(TAG, ptr, length, ESP_LOG_INFO);
        
        if( xQueueSend(Sdio_to_Tcp__Queue_Handle,ptr,0) == pdTRUE)   
        {
            printf("sdio send queue sussesful \n");
        }

        // echo data to SDIO host
        //esp32_sdio_write_data(ptr, length);
        
        // free SDIO buffer
        sdio_slave_recv_load_buf(handle);
    }
}

#if 1
int send_data_to_sdio(unsigned char* buf, unsigned int len)
{
    printf("send_data_to_sdio len %d\n",len);
    return esp32_sdio_write_data(buf, len);               
}

#else
static uint8_t sdio_buf[4096] = {0};
static void at_sdio_send_task(void* pvParameters)
{
    for (;;)
    {
        if( pdTRUE == xQueueReceive(Tcp_To_Sdio_Queue_Handle, sdio_buf, portMAX_DELAY))
        {
            printf("接收到tcp 的消息队列\n");
            // echo data to SDIO host
            esp32_sdio_write_data(sdio_buf, 4096);                   //4096
            memset(sdio_buf, 0, 128);
        }
    }
}
#endif

void sdio_init(void)
{
    // init slave driver
    esp32_sdio_slave_init();
    xTaskCreate(at_sdio_recv_task , "at_sdio_recv_task" , 4096 , NULL , 5 , NULL);
    //xTaskCreate(at_sdio_send_task , "at_sdio_send_task" , 4096 , NULL , 10, NULL);
}
