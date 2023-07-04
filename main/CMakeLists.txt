idf_component_register(SRCS  "wifi_task.c" "uart_task.c" "key_task.c" "app_main.c" 
                    INCLUDE_DIRS  "."
                    REQUIRES  comp_wifi  comp_ble app_update esp_http_client esp_https_ota protocomm
                    #EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem
                    )


                    #"ota_task.c"