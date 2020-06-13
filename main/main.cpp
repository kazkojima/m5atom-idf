/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "MPU6886.h"
#include "rotor.h"

#include "neopixel.h"

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
#define IN_CALIB 10000
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
    
    RotorIyVS RI (0.01, 1.0e-3, 1.0e-6);
    float c, si, sj, sk;

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
      //printf("ax: %f ay: %f az: %f\n", ax, ay, az);
      //printf("gx: %f gy: %f gz: %f\n", gx, gy, gz);
      // Convert NED frame into the normal frame.
      // Since gz will be used as the coefficient of bivector
                  // e1^e2 e1 <-> e2 makes the of gz minus.
      float oi, oj, ok;
      RI.Update (gy, gx, -gz, -ay, -ax, az,
                 c, si, sj, sk, oi, oj, ok);
      RI.Show ();
      count++;
    }
}

#define NEOPIXEL_PORT					27
#define NR_LED							25
#define NEOPIXEL_RMT_CHANNEL            RMT_CHANNEL_0

void led_task(void *arg)
{
    pixel_settings_t px;
    uint32_t pixels[NR_LED];
    int i;
    int rc;

    rc = neopixel_init(NEOPIXEL_PORT, NEOPIXEL_RMT_CHANNEL);
    printf("neopixel_init rc = %d", rc);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (i = 0; i < NR_LED; i++) {
        pixels[i] = 0;
    }
    px.pixels = (uint8_t *)pixels;
    px.pixel_count = NR_LED;
    strcpy(px.color_order, "GRB");

    memset(&px.timings, 0, sizeof(px.timings));
    px.timings.mark.level0 = 1;
    px.timings.space.level0 = 1;
    px.timings.mark.duration0 = 12;
    px.nbits = 24;
    px.timings.mark.duration1 = 14;
    px.timings.space.duration0 = 7;
    px.timings.space.duration1 = 16;
    px.timings.reset.duration0 = 600;
    px.timings.reset.duration1 = 600;

    px.brightness = 0x10;
    np_show(&px, NEOPIXEL_RMT_CHANNEL);
    
    int fact = 1;
    while (true) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        //ESP_LOGE("main", "fact = %d", fact);
        for (int j = 0; j < NR_LED; j++) {
            np_set_pixel_rgbw(&px, j , i, i, i, i);
        }
        np_show(&px, NEOPIXEL_RMT_CHANNEL);
        if (fact > 0) {
            i += 1;
        } else {
            i -= 1;
        }
        if (i == 255) {
            fact = -1;
        } else if ( i == 0 ) {
            fact = 1;
        }
    }
}

extern "C" void app_main()
{
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 1, NULL);
    xTaskCreate(led_task, "led_task", 8192, NULL, 2, NULL);

#if 1
    gpio_set_direction(GPIO_NUM_22, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_22, level);
        level = !level;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif
}
