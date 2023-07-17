#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Arduino_GFX.h>

#define SCREEN_W 800
#define SCREEN_H 480

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    14  /* DE */,
    41  /* VSYNC */,
    39  /* HSYNC */,
    42  /* DCLK */,
    45  /* R3 */,
    48  /* R4 */,
    47  /* R5 */,
    21  /* R6 */,
    40  /* R7 */,
    12  /* G2 */,
    11  /* G3 */,
    7   /* G4 */,
    15  /* G5 */,
    16  /* G6 */,
    13  /* G7 */,
    8   /* B3 */,
    3   /* B4 */,
    46  /* B5 */,
    9   /* B6 */,
    0   /* B7 */,
    1   /* hsync_polarity */,
    210 /* hsync_front_porch */,
    20   /* hsync_pulse_width */,
    46 /* hsync_back_porch */,
    1   /* vsync_polarity */,
    22  /* vsync_front_porch */,
    10   /* vsync_pulse_width */,
    23 /* vsync_back_porch */,
    0   /* pclk_active_neg */,
    32000000 /* prefer_speed */
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    800 /* width */, 480 /* height */, rgbpanel);

int w = SCREEN_W;
int h = SCREEN_H;

void setup()
{
    gfx->begin();
    gfx->fillScreen(WHITE);
    delay(1000);
}

void loop()
{
    gfx->fillScreen(WHITE);

    int shapeType = random(3);  // Generate a random number between 0 and 2
    
    switch (shapeType) {
        case 0:  // Create a random triangle
            {
                int x1 = random(w);
                int y1 = random(h);
                int x2 = random(w);
                int y2 = random(h);
                int x3 = random(w);
                int y3 = random(h);
                gfx->fillTriangle(x1, y1, x2, y2, x3, y3, gfx->color565(random(256), random(256), random(256)));
            }
            break;

        case 1:  // Create a random circle
            {
                int x = random(w);
                int y = random(h);
                int radius = random(10, 100);
                gfx->fillCircle(x, y, radius, gfx->color565(random(256), random(256), random(256)));
            }
            break;

        case 2:  // Create a random rectangle
            {
                int x = random(w);
                int y = random(h);
                int width = random(10, 200);
                int height = random(10, 200);
                gfx->fillRect(x, y, width, height, gfx->color565(random(256), random(256), random(256)));
            }
            break;
    }

    delay(1000);
}