#ifndef GPIO_H
#define GPIO_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

// Function prototypes
esp_err_t device_write_byte(uint8_t data);

#endif // GPIO_H
