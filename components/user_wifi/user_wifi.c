#include <stdio.h>
#include "user_wifi.h"

/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/gpio.h"
#include "freertos/event_groups.h"

#include "user_sdio.h"
#include "driver/uart.h"

#define PORT                        3333
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

static const char *TAG = "wifi";

#define EXAMPLE_ESP_WIFI_SSID      "myssid"
#define EXAMPLE_ESP_WIFI_PASS      "mypassword"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       4

#define BT_RST_N_R   36  /*wifi statu*/


int listen_sock;
int sock;

extern uint8_t mode_role;
extern uint8_t mode_change;
extern EventGroupHandle_t Event_Handle;
extern QueueHandle_t Tcp_To_Sdio_Queue_Handle;
extern QueueHandle_t Sdio_to_Tcp__Queue_Handle;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

char rx_buffer[4096] = {0};
static void do_retransmit(const int sock)
{
   int len;
   long temp_len = 0;

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
           // rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            //ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
            temp_len += len;
           // ESP_LOGI(TAG, "Received %d %ld", len, temp_len);
            //ESP_LOG_BUFFER_HEXDUMP("test", rx_buffer, len, ESP_LOG_INFO);
            //ESP_LOG_BUFFER_HEX(TAG, rx_buffer, len);

            //串口测试
            uart_write_bytes(UART_NUM_0, (char *)rx_buffer,len);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.

            /*tcp receive  sdio send carll*/
            // xEventGroupSetBits(Event_Handle,BIT1);   //事件标志组

            #if 0
            int sdio_len = send_data_to_sdio(rx_buffer, len);                               /*网口会有很高延时 会有粘包发生，包数过多时候sdio会死掉；sdio写会很浪费时间？？*/
            if(sdio_len > 0)
            {
                ESP_LOGI(TAG, "send_data_to_sdio %d ", sdio_len);
            }
            #endif

            #if 0
            if( pdTRUE == xQueueSend(Tcp_To_Sdio_Queue_Handle,rx_buffer,0) )
            {
                ESP_LOGI(TAG, "tcp send queue susessful");
            }
            #endif

            #if 0   //收到客户端数据再给发回去
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
            #endif
        }
    } while (len > 0);
}

static void tcp_server_receive_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    //int tcp_nodelay = 1;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }

    listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        //char sdio_buf[128];
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int)); 
        //setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &tcp_nodelay, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s \n", addr_str);

        do_retransmit(sock);

        ESP_LOGI("carll 1","carll_test socket shut down %d \n", sock);

        shutdown(sock, 0);  //modify_delete carll 
        close(sock);
        ESP_LOGI("carll 2","carll_test socket shut down %d \n", sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static void tcp_server_send_task(void* arg)
{
    char sdio_buf[128] = {0};
    for(;;)
    {
        /*sdio receive  tcp send*/
        if( pdTRUE == xQueueReceive(Sdio_to_Tcp__Queue_Handle,sdio_buf,100))
        {
            ESP_LOGI("carll","接送到sdio 发送的消息队列 \n");
            send(sock, sdio_buf, 128, 0);
        }  
    }
}

void send_data_tcp(char*buf, int len)
{
   //socks是否可以使用
    send(sock, buf, len, 0);
}


static void mode_switch(void* arg)
{
    for(;;)
    {
        if(mode_change == 1)
        {
            printf("mode_change INIT mode_role %d\r\n",mode_role);
            mode_change = 0;
            switch(mode_role)
            {
                case WIFI_CLOSE:
                    printf("wifi close\r\n");
                    ESP_ERROR_CHECK(esp_wifi_stop());
                    
                break;
                case WIFI_OPEN:
                    printf("wifi open\r\n");
                    ESP_ERROR_CHECK(esp_wifi_start());
                break;

                default:
                break;
            }  
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void wifi_init_softap(void)
{
    //ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    //esp_wifi_set_ps(WIFI_PS_NONE); //关闭wifi休眠模式

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

void ap_tcp_server_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_softap();

    xTaskCreate(tcp_server_receive_task, "tcp_server", 4096, (void*)AF_INET, 16, NULL);
    //xTaskCreate(tcp_server_send_task, "tcp_server_send_task", 4096, NULL, 1, NULL);
    //xTaskCreate(mode_switch, "mode_switch", 1024, NULL, 4, NULL);                                  //选择关闭或者开启wifi
}
