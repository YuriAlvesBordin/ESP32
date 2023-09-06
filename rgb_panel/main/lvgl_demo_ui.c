#include "lvgl.h"

lv_obj_t *canvas;

// Define the previous point
int16_t prev_x = -1;
int16_t prev_y = -1;

// Define the timeout value in milliseconds (e.g., 1000ms = 1 second)
#define TOUCH_TIMEOUT_MS 100

// Variables to track the last touch time
uint32_t last_touch_time = 0;

// Custom MAX macro
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

// Function to calculate rainbow color based on time
lv_color_t get_rainbow_color(uint32_t time_ms)
{
    // Define the period for a complete rainbow cycle (e.g., 5000ms = 5 seconds)
    uint32_t rainbow_period = 5000;

    // Calculate the hue value based on time
    uint16_t hue = (time_ms % rainbow_period) * 360 / rainbow_period;

    // Convert the hue to an RGB color
    return lv_color_hsv_to_rgb(hue, 100, 100);
}

void draw_rectangle_on_canvas(int16_t x, int16_t y, uint16_t pressure)
{
    // Get the current time
    uint32_t current_time = lv_tick_get();

    // Calculate the time elapsed since the last touch
    uint32_t time_elapsed = current_time - last_touch_time;

    // If the time elapsed is greater than the timeout, consider it a new line
    if (time_elapsed > TOUCH_TIMEOUT_MS) {
        prev_x = -1;
        prev_y = -1;
    }

    // Define the minimum and maximum sizes of the rectangle
    uint16_t min_rect_size = 5;   // Adjust this value for the minimum size
    uint16_t max_rect_size = 1000;  // Adjust this value for the maximum size

    // Calculate the scaling factor based on pressure
    float scale = ((float)pressure / 320000.0) * (max_rect_size - min_rect_size);

    // Calculate the size of the rectangle based on pressure and the range of sizes
    uint16_t rect_size = min_rect_size + (uint16_t)scale;

    // Calculate the position of the rectangle based on touch coordinates
    int16_t rect_x = x - rect_size / 2;
    int16_t rect_y = y - rect_size / 2;

    // Get the rainbow color based on the current time
    lv_color_t rainbow_color = get_rainbow_color(current_time);

    // If there is a previous point, interpolate between them
    if (prev_x != -1 && prev_y != -1) {
        int16_t dx = x - prev_x;
        int16_t dy = y - prev_y;
        int16_t steps = MAX(abs(dx), abs(dy)); // Number of steps to interpolate

        // Interpolate between the points and draw rectangles with rainbow color
        for (int i = 0; i < steps; i++) {
            float t = (float)i / (float)steps;
            int16_t interp_x = prev_x + (int16_t)(dx * t);
            int16_t interp_y = prev_y + (int16_t)(dy * t);
            
            lv_draw_rect_dsc_t rect_dsc;
            lv_draw_rect_dsc_init(&rect_dsc);
            rect_dsc.radius = 81;
            rect_dsc.bg_opa = LV_OPA_COVER;
            rect_dsc.bg_color = rainbow_color;
            lv_canvas_draw_rect(canvas, interp_x - rect_size / 2, interp_y - rect_size / 2, rect_size, rect_size, &rect_dsc);
        }
    }

    // Draw the final rectangle at the current touch point with rainbow color
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.radius = 81;
    rect_dsc.bg_opa = LV_OPA_COVER;
    rect_dsc.bg_color = rainbow_color;
    lv_canvas_draw_rect(canvas, rect_x, rect_y, rect_size, rect_size, &rect_dsc);

    // Refresh the canvas to show the drawn rectangle
    lv_obj_invalidate(canvas);

    // Update the previous point and last touch time
    prev_x = x;
    prev_y = y;
    last_touch_time = current_time;
}


void create_canvas()
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_scr_load(scr);

    canvas = lv_canvas_create(scr);
    lv_obj_set_size(canvas, LV_HOR_RES, LV_VER_RES);
    lv_obj_align(canvas, LV_ALIGN_CENTER, 0, 0);

    lv_color_t *buf = (lv_color_t *)lv_mem_alloc(LV_HOR_RES * LV_VER_RES * sizeof(lv_color_t));
    lv_canvas_set_buffer(canvas, buf, LV_HOR_RES, LV_VER_RES, LV_IMG_CF_TRUE_COLOR);

    // Set a background color for the canvas
    lv_canvas_fill_bg(canvas, lv_color_hex(0x000000), LV_OPA_COVER);
    
}

void lvgl_demo_ui()
{



create_canvas();
}
