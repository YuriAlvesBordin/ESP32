idf_component_register(
    SRCS 
    "main.c"

    "Hal/Programs/touch_ns2009.c"
    "Hal/Programs/Touch.c"
    "Hal/Programs/GPIO.c"
    "Hal/Programs/LCD.c"
    "Hal/Programs/LCD.c"

    "lvgl_demo_ui.c"
  
    INCLUDE_DIRS 
    "."
    "Hal/ConfigHeaders"
    "Hal/Headers"
)
