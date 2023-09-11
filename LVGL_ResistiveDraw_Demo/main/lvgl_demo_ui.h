#ifndef LVGL_DEMO_H
#define LVGL_DEMO_H

#include "lvgl.h"

extern lv_obj_t *canvas;

void draw_rectangle_on_canvas(int16_t x, int16_t y, uint16_t pressure);
void create_canvas();

#endif /* LVGL_DEMO_H */
