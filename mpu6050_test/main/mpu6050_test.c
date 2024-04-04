#include <stdio.h>
#include "mpu6050_driver.h"
#include "freertos/FreeRTOS.h"
#include "smoothing_filter.h"


void app_main(void)
{
    measurement_out_t measurement_out;
    int cnt = 0;

    mpu6050_init();

    while (1)
    {
        // mpu6050_get_value();
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
    
}
