/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "MPU6886.h"
#include "RotorIy.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define I2C_MASTER_SCL_IO               (gpio_num_t)21
#define I2C_MASTER_SDA_IO               (gpio_num_t)25
#define I2C_MASTER_NUM                  (i2c_port_t)I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000

#define I2C_PORT                        I2C_MASTER_NUM
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         (i2c_ack_type_t)0x0
#define NACK_VAL                        (i2c_ack_type_t)0x1

void _I2CInit(void)
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}

esp_err_t _I2CReadN(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|READ_BIT,
			  ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t _I2CWrite1(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t d = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, &d, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

#define GRAVITY_MSS     9.80665f

// For gyro
// 5 second
#define IN_CALIB 5000
#define CALIB_ALPHA 0.002f

#define FILTER_CONVERGE_COUNT 10000

void imu_task(void *arg)
{
    float gx, gy, gz;
    float ax, ay, az;
    float gx_offs = 0, gy_offs = 0, gz_offs = 0;
    int count = 0;

    vTaskDelay(100/portTICK_PERIOD_MS);
    _I2CInit();
    vTaskDelay(100/portTICK_PERIOD_MS);

    MPU6886 IMU;

    IMU.Init();
    
    RotorIyBV RIBV (0.01, 1.0e-3, 1.0e-6);
    //float c, si, sj, sk;
    float bc, bsi, bsj, bsk;

    for (;;) {
      IMU.getAccelData(&ax, &ay, &az);
      IMU.getGyroData(&gx, &gy, &gz);
      ax *= GRAVITY_MSS;
      ay *= GRAVITY_MSS;
      az *= GRAVITY_MSS;
      //printf("ax: %f ay: %f az: %f\n", ax, ay, az);
      //printf("gx: %f gy: %f gz: %f\n", gx, gy, gz);
	  // gyro offset calibration needed
	  if (count < IN_CALIB) {
	      gx_offs = (1-CALIB_ALPHA)*gx_offs + CALIB_ALPHA*gx;
	      gy_offs = (1-CALIB_ALPHA)*gy_offs + CALIB_ALPHA*gy;
	      gz_offs = (1-CALIB_ALPHA)*gz_offs + CALIB_ALPHA*gz;
	      count++;
	      continue;
	    }

	  gx -= gx_offs; gy -= gy_offs; gz -= gz_offs;
	  RIBV.UpdateIMU (gx, gy, gz, ax, ay, az, bc, bsi, bsj, bsk);
      RIBV.Show ();
      count++;
    }
}

extern "C" void app_main()
{
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 1, NULL);

    gpio_set_direction(GPIO_NUM_22, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_22, level);
        level = !level;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
