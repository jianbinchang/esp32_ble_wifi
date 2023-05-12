#include "wifi_task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/uart.h"
#include "comp_wifi.h"

#include "freertos/queue.h"

#define PORT                        3333
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

int listen_sock;
int sock;
static const char* TAG = "wifi_task";
static char rx_buffer[4096] = {0};
QueueHandle_t g_Event_Handle = NULL;

void send_data_tcp(char*buf, int len)
{
   //socks是否可以使用
    send(sock, buf, len, 0);
}

static void do_retransmit(const int sock)
{
   int len;
   //long temp_len = 0;
    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
           //temp_len += len;
           // ESP_LOGI(TAG, "Received %d %ld", len, temp_len);

            //串口测试
            uart_write_bytes(UART_NUM_0, (char *)rx_buffer,len);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.

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


void send_que_to_wifi(const void* p_val)
{
    if( g_Event_Handle !=  NULL)
    {
           xQueueSend(g_Event_Handle, p_val, 0);
    }
}

static void wifi_status_switch(void* arg)
{
    uint8_t mode_statu = 0;
    for(;;)
    {
        if( pdTRUE == xQueueReceive(g_Event_Handle, &mode_statu, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "xQueueReceive %d \n",mode_statu);
             
             switch(mode_statu)
            {
                case WIFI_CLOSE:
                    ESP_LOGI(TAG, "wifi close\n");
                    ESP_ERROR_CHECK(esp_wifi_stop());
                break;
                case WIFI_OPEN:
                    ESP_LOGI(TAG, "wifi open\n");
                    ESP_ERROR_CHECK(esp_wifi_start());
                break;

                default:
                break;
            } 
        }
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void ap_tcp_server_wifi(void)
{
    wifi_init_softap();   //wifi 默认是关闭

    g_Event_Handle = xQueueCreate(1,1);
    if(g_Event_Handle)
    {
         ESP_LOGI(TAG, "g_Event_Handle creat ok\n");
    }

    xTaskCreate(tcp_server_receive_task, "tcp_server", 4096, (void*)AF_INET, 16, NULL);
    xTaskCreate(wifi_status_switch, "mode_switch", 2048, NULL, 4, NULL);                                  //选择关闭或者开启wifi
}