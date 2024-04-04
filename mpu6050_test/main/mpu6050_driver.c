#include "mpu6050_driver.h"
#include "esp_timer.h"
#include "smoothing_filter.h"

#define IMU_DATA_SIZE_BYTES 14
#define BUFFER_SAMPLES 5
#define OFFSET_LOOP 1000
#define ACCEL_THRESHOLD 100
#define GYRO_THRESHOLD 7

// Handle for the timer that is used to sample the IMU data
esp_timer_handle_t i2cTimerHandle;

// Buffer to store I2C read data from the IMU
uint8_t imu_data[IMU_DATA_SIZE_BYTES];

// Buffers to hold 5 samples of data from IMU in order to do
// the filtering
int16_t accel_x_buffer[BUFFER_SAMPLES] = {};
int16_t accel_y_buffer[BUFFER_SAMPLES] ={};
int16_t accel_z_buffer[BUFFER_SAMPLES] = {};
int16_t gyro_x_buffer[BUFFER_SAMPLES] = {};
int16_t gyro_y_buffer[BUFFER_SAMPLES] = {};
int16_t gyro_z_buffer[BUFFER_SAMPLES] = {};
int16_t temp_data_buffer[BUFFER_SAMPLES] = {};

// Offset values to account for initial bias
int16_t gyro_x_offset = 0;
int16_t gyro_y_offset = 0;
int16_t gyro_z_offset = 0;
int16_t temp_data_offset = 0;

// The final acceleration values that can be used for
// calculations
int16_t accel_x_output;
int16_t accel_y_output;
int16_t accel_z_output;

// The filtered accelerations data from the IMU
int16_t accel_x_output_filtered;
int16_t accel_y_output_filtered;
int16_t accel_z_output_filtered;

// Previous run's filtered acceleration value
int16_t accel_x_output_old;
int16_t accel_y_output_old;
int16_t accel_z_output_old;

// Final output acceleration values that can be used in
// calculations
int16_t gyro_x_output;
int16_t gyro_y_output;
int16_t gyro_z_output;
int16_t temp_data_output;

void timerCallback(void* arg);

static const uint8_t mpu6050_init_cmd[11][2] = {
    //{MPU6050_RA_PWR_MGMT_1, 0x80}, // PWR_MGMT_1, DEVICE_RESET  
    // need wait 
    {MPU6050_RA_PWR_MGMT_1, 0x00}, // cleat SLEEP
    {MPU6050_RA_GYRO_CONFIG, 0x18}, // Gyroscope Full Scale Range = ± 2000 °/s
    {MPU6050_RA_ACCEL_CONFIG, 0x00}, // Accelerometer Full Scale Range = ± 2g 
    {MPU6050_RA_INT_ENABLE, 0x00}, // Interrupt Enable.disenable 
    {MPU6050_RA_USER_CTRL, 0x00}, // User Control.auxiliary I2C are logically driven by the primary I2C bus
    {MPU6050_RA_FIFO_EN, 0x00}, // FIFO Enable.disenable  
    {MPU6050_RA_SMPLRT_DIV, 0x63}, // Sample Rate Divider.Sample Rate = 1KHz / (1 + 99)  
    {MPU6050_RA_CONFIG, 0x0B}, // 0x13 EXT_SYNC_SET = GYRO_XOUT_L[0]; Bandwidth = 3
    {MPU6050_RA_PWR_MGMT_1, 0x01}, // Power Management 1.PLL with X axis gyroscope reference
    {MPU6050_RA_PWR_MGMT_2, 0x00}  // Power Management 2
};

static esp_err_t  i2c_master_read_slave(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
* @brief 初始化 mpu6050
*/
esp_err_t mpu6050_init()
{
    esp_err_t esp_err;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_I2C_SDA,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MPU6050_I2C_SCL,         // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ,  // select frequency specific to your project
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err = i2c_param_config(MPU6050_I2C_PORT_NUM, &conf);
    printf("i2c_param_config: %d \n", esp_err);

    esp_err = i2c_driver_install(MPU6050_I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
    printf("i2c_driver_install: %d \n", esp_err);

    uint8_t reset_command[1][2] = {{MPU6050_RA_PWR_MGMT_1, 0x80}};

    esp_err = i2c_master_write_slave(MPU6050_I2C_PORT_NUM, reset_command, 2);

    vTaskDelay(200 / portTICK_PERIOD_MS);

    for (size_t i = 0; i < 11 && esp_err == ESP_OK; i++)
    {
        esp_err = i2c_master_write_slave(MPU6050_I2C_PORT_NUM, mpu6050_init_cmd[i], 2);
        if (i == 0)
            vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    printf("mpu6050_init_cmd: %d \n", esp_err);

    // Following settup we should find the offset
    int64_t gyro_x_offset_temp = 0;
    int64_t gyro_y_offset_temp = 0;
    int64_t gyro_z_offset_temp = 0;
    int64_t temp_data_offset_temp = 0;

    vTaskDelay(200 / portTICK_PERIOD_MS);

    for( unsigned int i = 0; i < OFFSET_LOOP && esp_err == ESP_OK; ++i )
    {
        esp_err = i2c_master_read_slave( MPU6050_I2C_PORT_NUM, MPU6050_RA_ACCEL_XOUT_H, imu_data, 14 );
        
        if( esp_err == ESP_OK )
        {
            temp_data_offset_temp += (int16_t)(imu_data[6]<<8 | imu_data[7]);
            gyro_x_offset_temp += (int16_t)(imu_data[8]<<8 | imu_data[9]);
            gyro_y_offset_temp += (int16_t)(imu_data[10]<<8 | imu_data[11]);
            gyro_z_offset_temp += (int16_t)(imu_data[12]<<8 | imu_data[13]);
        }
    }

    temp_data_offset = (int16_t)(temp_data_offset_temp/OFFSET_LOOP);
    gyro_x_offset = (int16_t)(gyro_x_offset_temp/OFFSET_LOOP);
    gyro_y_offset = (int16_t)(gyro_y_offset_temp/OFFSET_LOOP);
    gyro_z_offset = (int16_t)(gyro_z_offset_temp/OFFSET_LOOP);

    printf("temp_data_offset: %d\n", temp_data_offset);
    printf("gyro_x_offset: %d\n", gyro_x_offset);
    printf("gyro_y_offset: %d\n", gyro_y_offset);
    printf("gyro_z_offset: %d\n", gyro_z_offset);

    printf("Configuring timer for periodic measurements\n");
    esp_timer_create_args_t i2cTimerArgs;
    i2cTimerArgs.name = "I2C Data Collection Timer";
    i2cTimerArgs.dispatch_method = ESP_TIMER_TASK;
    i2cTimerArgs.callback = timerCallback;

    esp_timer_create( &i2cTimerArgs, &i2cTimerHandle );

    // Should configure the timer for every 100ms
    esp_timer_start_periodic( i2cTimerHandle, 100000 );

    return esp_err;
}

/**
* @brief 读取加速度计、温度和陀螺仪数据
*/
void mpu6050_get_value()
{
    
}

void timerCallback(void* arg)
{
    // Read the accelerations and gyro data from the IMU
    esp_err_t status = i2c_master_read_slave( MPU6050_I2C_PORT_NUM, MPU6050_RA_ACCEL_XOUT_H, imu_data, 14 );

    if( status == ESP_OK )
    {
        // Shift the data down in the buffer
        accel_x_buffer[0] = accel_x_buffer[1];
        accel_x_buffer[1] = accel_x_buffer[2];
        accel_x_buffer[2] = accel_x_buffer[3];
        accel_x_buffer[3] = accel_x_buffer[4];
        accel_x_buffer[4] = (int16_t)(imu_data[0]<<8 | imu_data[1]);

        // Shift the data down in the buffer
        accel_y_buffer[0] = accel_y_buffer[1];
        accel_y_buffer[1] = accel_y_buffer[2];
        accel_y_buffer[2] = accel_y_buffer[3];
        accel_y_buffer[3] = accel_y_buffer[4];
        accel_y_buffer[4] = (int16_t)(imu_data[2]<<8 | imu_data[3]);

        // Shift the data down in the buffer
        accel_z_buffer[0] = accel_z_buffer[1];
        accel_z_buffer[1] = accel_z_buffer[2];
        accel_z_buffer[2] = accel_z_buffer[3];
        accel_z_buffer[3] = accel_z_buffer[4];
        accel_z_buffer[4] = (int16_t)(imu_data[4]<<8 | imu_data[5]);
        
        // Shift the data down in the buffer
        temp_data_buffer[0] = temp_data_buffer[1];
        temp_data_buffer[1] = temp_data_buffer[2];
        temp_data_buffer[2] = temp_data_buffer[3];
        temp_data_buffer[3] = temp_data_buffer[4];
        temp_data_buffer[4] = (int16_t)(imu_data[6]<<8 | imu_data[7]) - temp_data_offset;

        // Shift the data down in the buffer
        gyro_x_buffer[0] = gyro_x_buffer[1];
        gyro_x_buffer[1] = gyro_x_buffer[2];
        gyro_x_buffer[2] = gyro_x_buffer[3];
        gyro_x_buffer[3] = gyro_x_buffer[4];
        gyro_x_buffer[4] = (int16_t)(imu_data[8]<<8 | imu_data[9]) - gyro_x_offset;

        // Shift the data down in the buffer
        gyro_y_buffer[0] = gyro_y_buffer[1];
        gyro_y_buffer[1] = gyro_y_buffer[2];
        gyro_y_buffer[2] = gyro_y_buffer[3];
        gyro_y_buffer[3] = gyro_y_buffer[4];
        gyro_y_buffer[4] = (int16_t)(imu_data[10]<<8 | imu_data[11]) - gyro_y_offset;

        // Shift the data down in the buffer
        gyro_z_buffer[0] = gyro_z_buffer[1];
        gyro_z_buffer[1] = gyro_z_buffer[2];
        gyro_z_buffer[2] = gyro_z_buffer[3];
        gyro_z_buffer[3] = gyro_z_buffer[4];
        gyro_z_buffer[4] = (int16_t)(imu_data[12]<<8 | imu_data[13]) - gyro_z_offset;

        // See if this takes too long
        FilterData(accel_x_buffer, BUFFER_SAMPLES, &accel_x_output_filtered);
        FilterData(accel_y_buffer, BUFFER_SAMPLES, &accel_y_output_filtered);
        FilterData(accel_z_buffer, BUFFER_SAMPLES, &accel_z_output_filtered);
        FilterData(temp_data_buffer, BUFFER_SAMPLES, &temp_data_output);
        FilterData(gyro_x_buffer, BUFFER_SAMPLES, &gyro_x_output);
        FilterData(gyro_y_buffer, BUFFER_SAMPLES, &gyro_y_output);
        FilterData(gyro_z_buffer, BUFFER_SAMPLES, &gyro_z_output);

        accel_x_output = accel_x_output_filtered - accel_x_output_old;
        accel_y_output = accel_y_output_filtered - accel_y_output_old;
        accel_z_output = accel_z_output_filtered - accel_z_output_old;

        accel_x_output_old = accel_x_output_filtered;
        accel_y_output_old = accel_y_output_filtered;
        accel_z_output_old = accel_z_output_filtered;

        // Make sure the data is above a minimum threshold. This prevents tiny
        // movement from effecting our data
        accel_x_output = (accel_x_output < ACCEL_THRESHOLD) ? 0 : accel_x_output;
        accel_y_output = (accel_y_output < ACCEL_THRESHOLD) ? 0 : accel_y_output;
        accel_z_output = (accel_z_output < ACCEL_THRESHOLD) ? 0 : accel_z_output;
        gyro_x_output = (gyro_x_output < GYRO_THRESHOLD) ? 0 : gyro_x_output;
        gyro_y_output = (gyro_y_output < GYRO_THRESHOLD) ? 0 : gyro_y_output;
        gyro_z_output = (gyro_z_output < GYRO_THRESHOLD) ? 0 : gyro_z_output;

        printf("accel_x_output: %d\n", accel_x_output);
        printf("accel_y_output: %d\n", accel_y_output);
        printf("accel_z_output: %d\n", accel_z_output);
        printf("gyro_x_output: %d\n", gyro_x_output);
        printf("gyro_y_output: %d\n", gyro_y_output);
        printf("gyro_z_output: %d\n", gyro_z_output);

    }
    else
    {
        printf("I2C read error\n");
    }
}