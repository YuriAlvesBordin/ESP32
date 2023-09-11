#ifndef LCD_MAIN_H
#define LCD_MAIN_H

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function to initialize the LCD and LVGL
void lcd_main();

void lvgl_ticktimer();

extern lv_disp_t *disp;

#ifdef __cplusplus
}
#endif

#endif /* LCD_MAIN_H */
