
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "driver/i2c.h"
#include "touch_ns2009.h"
#include "lvgl_demo_ui.h"

static const char *TAG = "Touch";

void touch_interrupt_handler(lv_indev_drv_t * indevDriver, lv_indev_data_t * indevData)
{
    int pos[2];
    int pressure = ns2009_pos(pos);

    // Handle touch press and release
    if (pressure > 300 && pressure < 4000 && pos[0] > 0 && pos[1] > 0) {
        // Adjust the coordinates based on your display's orientation and calibration
        indevData->point.x = pos[0]; // Adjust X coordinate
        indevData->point.y = pos[1]; // Adjust Y coordinate
        indevData->state = LV_INDEV_STATE_PR;

        if (pressure > 3000){
            ESP_LOGW(TAG, ": (%d, %d)", indevData->point.x, indevData->point.y);
            ESP_LOGW(TAG, "Pressure: %d\n", pressure);
            ESP_LOGW(TAG, "High Pressure Detected on the Touch Screen");
        }

        draw_rectangle_on_canvas(indevData->point.x, indevData->point.y, pressure);
        
        // Print touch data for debugging
        ESP_LOGI(TAG, "Touch: (%d, %d)", indevData->point.x, indevData->point.y);
        printf("Pressure: %d\n", pressure);
    } else {
        indevData->state = LV_INDEV_STATE_REL;
    }
}

void touch_main(lv_disp_t *disp)
{
    // Create and initialize the touch input device driver
    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_interrupt_handler;

    // Associate the display with the input device driver
    indev_drv.disp = disp;

    // Register the input device driver with LVGL
    lv_indev_drv_register(&indev_drv);
}

