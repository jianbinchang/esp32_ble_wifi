#include "key_task.h"
#include "string.h"       

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "comp_wifi.h"   
#include "esp_log.h"

#include "driver/gpio.h"

extern QueueHandle_t g_Event_Handle;

// static void IRAM_ATTR gpio_isr_handler(void* arg)
// {
//     uint8_t mode_role;
//     uint32_t gpio_num = (uint32_t) arg;
//     if ( gpio_get_level(gpio_num) == 1)
//     {
//        mode_role = WIFI_OPEN;
//     }
//     else
//     {
//         mode_role = WIFI_CLOSE;
//     }
//     send_que_to_wifi(&mode_role);
// }

void gpio_init(void)
{
     /*init gpio*/
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    memset(&io_conf,0,sizeof(io_conf));
    gpio_pad_select_gpio(AP_WAKE_BT_R);
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;                 /*双边沿 GPIO_INTR_ANYEDGE*/
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //install gpio isr service
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(AP_WAKE_BT_R, gpio_isr_handler, (void*) AP_WAKE_BT_R);
    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);
    ESP_LOGI("key_task","GPIO INIT\r\n");                                   /*会输出debug*/

}

static void key_scan(void* arg)
{   
    uint8_t status_last;
    uint8_t status = 0;
    vTaskDelay(500 / portTICK_PERIOD_MS);

    for(;;)
    {
        status_last = status;
        status = gpio_get_level(AP_WAKE_BT_R);
        if(status != status_last)
        {
            ESP_LOGI("key_task", "xQueueSend\n");
            //send_que_to_wifi(&status);              //这种形式需要包含任务头文件，同一层任务不好相互调用
            if(g_Event_Handle) 
            {
                xQueueSend(g_Event_Handle, &status, 0);
            }
           
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    } 
}


void keytask_start(void)
{
    gpio_init();
    xTaskCreate(key_scan, "key_scan", 2048, NULL, 1, NULL);
}
