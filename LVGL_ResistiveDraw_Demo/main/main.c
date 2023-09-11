
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "touch_ns2009.h"
#include "Touch.h"
#include "LCD.h"
#include "GPIO.h"
#include "lvgl_demo_ui.h"

static void i2c_init()
{
    // Configure I2C communication here
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 17, // GPIO pin for SDA
        .scl_io_num = 18, // GPIO pin for SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000 // I2C clock speed
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void app_main(void)
{
    ESP_LOGI("Main", "Init I2C");
    i2c_init();

    ESP_LOGI("Main", "LCD");
    lcd_main();

    ESP_LOGI("Main", "IO Expander MSG");
    device_write_byte(0b00000101);

    ESP_LOGI("Main", "First LVGL function");
    create_canvas();

    ESP_LOGI("Main", "Init Touch Indev");
    touch_main(disp);

    ESP_LOGI("Main", "Install LVGL tick timer");
    lvgl_ticktimer();
}
