#ifndef TOUCH_H
#define TOUCH_H

#include "lvgl.h"

// Function prototypes
void touch_interrupt_handler(lv_indev_drv_t *indevDriver, lv_indev_data_t *indevData);
void touch_main(lv_disp_t *disp);

#endif // TOUCH_H
