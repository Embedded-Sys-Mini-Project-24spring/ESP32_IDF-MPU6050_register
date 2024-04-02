#include <stdio.h>
#include "esp_err.h"
#include "mpu6050_driver.h"
#include "freertos/FreeRTOS.h"


void app_main(void)
{
    measurement_out_t measurement_out;
    int cnt = 0;

    esp_err_t err = mpu6050_init();
    // while (err != ESP_OK) {
    //     printf("Fuck, init fail \n");
    // }

    while (1)
    {
        measurement_out = mpu6050_get_value();
        printf("accel_xout:%d\t;", measurement_out.accel_out.accel_xout);
        printf("accel_yout:%d\t;", measurement_out.accel_out.accel_yout);
        printf("accel_zout:%d\t;", measurement_out.accel_out.accel_zout);
        printf("gyro_xout:%d\t;", measurement_out.gyro_out.gyro_xout);
        printf("gyro_yout:%d\t;", measurement_out.gyro_out.gyro_yout);
        printf("gyro_zout:%d\t;", measurement_out.gyro_out.gyro_zout);
        printf("temp:%d\t;", measurement_out.temp_out.temp_xout);
        printf("cnt:%d\n", cnt++);
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
    
}
