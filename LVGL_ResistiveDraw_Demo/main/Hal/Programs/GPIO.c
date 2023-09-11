
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

//Expander inital data
uint8_t data = 0b00000101;

//IO Expander write data
esp_err_t device_write_byte(uint8_t data)
{
    int ret;
    ESP_LOGI("IO Expander", "%d",data);
    uint8_t write_buf[1] = {data}; // the last bit is the pin state

    ret = i2c_master_write_to_device(0, 0x20, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
  
    return ret;
}
