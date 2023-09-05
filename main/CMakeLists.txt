idf_component_register(SRCS "ota_task.c" "wifi_task.c" "uart_task.c" "key_task.c" "app_main.c" 
                    INCLUDE_DIRS  "."
                    REQUIRES  comp_wifi  comp_ble app_update esp_http_client esp_https_ota protocomm comp_imu
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem
                    )
