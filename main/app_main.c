#include "nvs.h"
#include "nvs_flash.h"
#include "user_gpio.h"
#include "user_sdio.h"
#include "user_ble.h"
#include  "user_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "string.h"

#define SDIO_WIFI 0

EventGroupHandle_t Event_Handle = NULL;
QueueHandle_t Queue_Handle, My_Queue_Handle;

 void app_main(void)
{
    esp_err_t ret;
    
    //init gpio
    gpio_init();

    //Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    user_ble_init();

    spp_uart_init();

 //wifi功能暂时没有启用   
#if SDIO_WIFI
     //creat event group
    Event_Handle = xEventGroupCreate();
    if(Event_Handle == NULL)
    {
        printf("Event_Handle error");
    }

    Queue_Handle = xQueueCreate(2,128);     //creat 2 128 byte queue
    if(Queue_Handle == NULL)
    {
         printf("Queue_Handle error");
    }

    My_Queue_Handle = xQueueCreate(2,128);  //creat 2 128 byte queue
    if(My_Queue_Handle == NULL)
    {
         printf("My_Queue_Handle error");
    }

    ap_tcp_server_wifi();

    sdio_init();
#endif

#if 0
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);/* code */
        #if 0
        EventBits_t r_event;                                    /* 定义一个事件接收变量 */ 
                r_event = xEventGroupWaitBits(Event_Handle,     /* 事件对象句柄 */ 
                              BIT0,                        /* 接收任务感兴趣的事件 */ 
                              pdTRUE,                           /* 退出时清除事件位 */ 
                              pdTRUE,                           /* 满足感兴趣的所有事件 */ 
                              portMAX_DELAY);                   /* 指定超时事件,一直等 */ 
  
        if (r_event == BIT0) 
        { 
            /* 如果接收完成并且正确 */ 
            printf ( "BIT0 BIT1 RECEIVE_CARLL\n"); 
        } 
        else 
        {
            printf ( "事件错误！\n"); 
        }
        #endif

        #if 0
        char rx_buffer[128];
        if( pdTRUE == xQueueReceive(Queue_Handle,rx_buffer,100) )
        {
            //printf("tcp send queue susessful");
            ESP_LOGI("carll", "接收到 queue susessful %c",rx_buffer[1]);
        }
        //ESP_LOGI("carll","接收到 queue fail\n");
        if(rx_buffer[1] == '2')
        {
            if( pdTRUE == xQueueSend(My_Queue_Handle,rx_buffer,0) )  //发送不会阻塞 carll
            {
                ESP_LOGI("carll","发送 queue sucessful\n");
                memset(rx_buffer,0,128);
            }   
            //ESP_LOGI("carll","发送 queue fail\n");
        }       
        #endif
    }
#endif
    return;
}