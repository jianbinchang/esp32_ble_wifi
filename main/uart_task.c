#include "string.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "comp_ble.h"

QueueHandle_t spp_uart_queue = NULL;

void send_data_tcp(char*buf, int len);  //test
void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;) 
    {
        //Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                if (event.size) {
                    uint8_t * temp = NULL;
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    if(temp == NULL){
                        ESP_LOGE("UART to BLE", "malloc failed,%s L#%d\n", __func__, __LINE__);
                        break;
                    }
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);
                    esp_log_buffer_hex("UART_RECEV", temp, event.size);

                    if((*temp == '#') && (*(temp+1) == '#') && (is_connect_ap = true))
                    {
                        comp_esp_ble_gatts_send_notify(temp, event.size); 
                    }
                    else if((*temp == '$') && (*(temp+1) == '$'))
                    {
                        send_data_tcp((char*)(temp+2), event.size-2);
                    }
                    else if(is_connect == true)
                    {
                        comp_esp_ble_gattc_write_char(temp, event.size);
                    }

                    if(strncmp((char *)temp, "peidui",6) == 0)
                    {
                        //printf("peidui \r\n");
                        save_remote_bound_add();
                    }
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,  //UART_SCLK_DEFAULT
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 20, &spp_uart_queue, 0);
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 2048, (void*)UART_NUM_0, 8, NULL);
}