#include "string.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "comp_ble.h"
#include "ota_task.h"
#include "eulersensor.h"
#define TAG "uart_task"
#define BUF_SIZE 1024
#define RD_BUF_SIZE (BUF_SIZE)

QueueHandle_t spp_uart_queue = NULL;
QueueHandle_t uart1_queue = NULL;
static size_t read_num_sum =0;
size_t buffered_size = 0; //test

void send_data_tcp(char*buf, int len);


void uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t * temp = NULL;
    uint8_t * read_temp;
    read_temp = temp = (uint8_t *)malloc(sizeof(uint8_t)*4096);

    if(temp == NULL)
    {
        while (1)
        {
             ESP_LOGE("UART to BLE", "malloc failed,%s L#%d\n", __func__, __LINE__);
             vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }

    for (;;) 
    {
        if (xQueueReceive(spp_uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            //ESP_LOGI("carll 1", "uart recv event %d", event.timeout_flag);
            switch (event.type) {
            case UART_DATA:
                if (event.size) {

                    size_t read_len = uart_read_bytes(UART_NUM_0, read_temp, event.size, portMAX_DELAY);
                    read_temp += read_len;         /*指针偏移*/
                    read_num_sum += read_len;      /*总共读取的长度*/

                    ESP_LOGI("carll 1","event.size %d  read_num_sum %d\n", event.size, read_num_sum);

                    if(read_len ==  120)           /*120个字节会触发UART_DATA 中断 不可以发送120个字节的整数倍*/
                    {
                         break;
                    }

                    //esp_log_buffer_hex("carll ", temp, read_num_sum);      /*大量发送数据会发生死机*/

                    if((*temp == '#') && (*(temp+1) == '#') /*&& (is_connect_ap == true)*/)    //##是通过蓝牙向手机发送数据
                    {
                        comp_esp_ble_gatts_send_notify(temp+2, read_num_sum-2);
                    }
                    else if((*temp == '$') && (*(temp+1) == '$'))                              //$$是通过网口向app发送数据
                    {
                        send_data_tcp((char*)(temp+2), read_num_sum-2);
                    }
                    else if(strncmp((char *)temp, "peidui",6) == 0)
                    {
                        save_remote_bound_add();
                    }
                    else if(strncmp((char *)temp, "ota_updata",10) == 0)
                    {
                        set_ota_flag();
                    }
                    // else if(strncmp((char *)temp, "gyr_check",9) == 0)
                    // {
                    //     is_en_check_gyr_slot();
                    // }
                    // else if(strncmp((char *)temp, "mag_check",9) == 0)
                    // {
                    //     is_en_check_amg_slot();
                    // }

                    comp_esp_ble_gattc_write_char(temp, read_num_sum);                           //向本体蓝牙发送数据
            
                    read_temp = temp;
                    memset(temp, 0x0, read_num_sum);
                    read_num_sum = 0;
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(spp_uart_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(spp_uart_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(UART_NUM_0, &buffered_size);
                int pos = uart_pattern_pop_pos(UART_NUM_0);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(UART_NUM_0);
                } else {
                    ESP_LOGI(TAG, "uart UART_PATTERN_DET");
                    // char temp[1024];
                    // uart_read_bytes(UART_NUM_0, temp, pos, 100 / portTICK_PERIOD_MS);
                    // uint8_t pat[3 + 1];
                    // memset(pat, 0, sizeof(pat));
                    // uart_read_bytes(UART_NUM_0, pat, 3, 100 / portTICK_PERIOD_MS);
                    // ESP_LOGI(TAG, "read data: %s", temp);
                    // ESP_LOGI(TAG, "read pat : %s", pat);
                }
                break;
            default:
                break;
            }
        }
    }

    free(temp);
    temp = NULL;
    vTaskDelete(NULL);
}

/**
*********************************************************************************************************************
* @fn          :  get_check_xor()
* @brief       :  检验和计算
* @param[in]   :  none
* @return      :  找到返回,位置指针，找不到返回NULL
* @note        :  revision history:
* @note        :  version 1.0,written by GAL,2018-05-15
**********************************************************************************************************************/
static uint8_t get_check_xor(const uint8_t *check_buf, const uint16_t buf_len)
{
    uint16_t cnt = 0 ;
    uint8_t return_check_value = 0;

    for(cnt = 0 ; cnt < buf_len ; cnt ++)        //计算校验
    {
        return_check_value ^=  check_buf[ cnt ] ;
    }
    return  return_check_value ;
}

/**
 * 六轴数据解包组包发送
 * 
*/
void para_pack_send(uint8_t* buf, int len)
{   
    //六轴变量
    float roll, pitch, yaw;
    uint8_t offset = 0;
    uint8_t imu_temp[30]={0};
    uint8_t xor_val;

    if(len > 30)  return;
    if(buf[0] != 0xFE || buf[1] != 0x03) return;

    memcpy(imu_temp, buf, len);
    imu_get_atti(&roll, &pitch, &yaw);
  
    imu_temp[offset++] = 0xFE;
    char * p = (char*)&roll;
    imu_temp[offset++] = *p;
    imu_temp[offset++] = *(p+1);
    imu_temp[offset++] = *(p+2);
    imu_temp[offset++] = *(p+3);

    p = (char*)&pitch;
    imu_temp[offset++] = *p;
    imu_temp[offset++] = *(p+1);
    imu_temp[offset++] = *(p+2);
    imu_temp[offset++] = *(p+3);

    p = (char*)&yaw;
    imu_temp[offset++] = *p;
    imu_temp[offset++] = *(p+1);
    imu_temp[offset++] = *(p+2);
    imu_temp[offset++] = *(p+3);

    xor_val = get_check_xor(imu_temp, offset);
    imu_temp[offset++] = xor_val;

    uart_write_bytes(UART_NUM_1, (char *)imu_temp, offset);/*carll*/
}


/**
 * 串口1 与核心板传递姿态信息
*/
static void uart_event_task1(void *pvParameters)
{
    uart_event_t event;

    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_1);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI("UART_NUM_1","event.size %d\n", event.size);
                    para_pack_send(dtmp, event.size);  //carll add

                    if(strncmp((char *)dtmp, "gyr_check",9) == 0)
                    {
                        is_en_check_gyr_slot();
                    }
                    else if(strncmp((char *)dtmp, "mag_check",9) == 0)
                    {
                        is_en_check_amg_slot();
                    }

                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    break;
                //Others
                default:
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}


void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,              //460800
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,  //UART_SCLK_DEFAULT
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 4096, 20, &spp_uart_queue, 0);  //8192  20
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 4096, (void*)UART_NUM_0, 8, NULL);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
     uart_config_t uart_config1 = {
         .baud_rate = 115200,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .source_clk = UART_SCLK_APB,  //UART_SCLK_REF_TICK  这个时钟会一个字节中断
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
     };
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE, BUF_SIZE, 20, &uart1_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config1));

    //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 26, 0, 0, 0));

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task1, "uart_event_task1", 2048, NULL, 12, NULL);

    imu_regist_uart_handle(uart_write_bytes);  //注册imu 串口发送函数
}