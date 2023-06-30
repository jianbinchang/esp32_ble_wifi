#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "string.h"

#include "comp_sdio.h"
#include "comp_ble.h"
#include "comp_wifi.h"

#include "key_task.h"
#include "uart_task.h"
#include "wifi_task.h"

#define SDIO 0

 void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    keytask_start();

    user_ble_init();

    spp_uart_init();

    ap_tcp_server_wifi();

#if SDIO
    sdio_init();
#endif

    return;
}
