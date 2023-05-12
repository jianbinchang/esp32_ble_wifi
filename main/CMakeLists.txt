idf_component_register(SRCS "wifi_task.c" "uart_task.c" "key_task.c" "app_main.c" 
                    INCLUDE_DIRS  "."
                    REQUIRES  comp_wifi comp_sdio comp_ble
                    )
