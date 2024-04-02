idf_component_register(SRCS "mpu6050_test.c" "mpu6050_driver.c" "smoothing_filter.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer driver)
                    