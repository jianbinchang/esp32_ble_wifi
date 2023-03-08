idf_component_register(SRCS "app_main.c" 
                    INCLUDE_DIRS  "."
                    REQUIRES user_gpio user_wifi user_sdio user_ble
                    )
