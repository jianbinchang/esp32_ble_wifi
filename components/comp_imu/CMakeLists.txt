idf_component_register(SRCS "esp32_i2c.c" "eulersensor.c" "jcc_ellipse.c" "jcc_ellipsoid.c" "jcc_IMU_lite.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash)
