#include <stdio.h>
#include "esp_err.h"
#include "mpu6050_driver.h"
#include "freertos/FreeRTOS.h"
#include "smoothing_filter.h"


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
        // mpu6050_get_value();
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
    
}
