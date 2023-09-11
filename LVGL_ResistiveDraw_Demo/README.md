| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# RGB LCD Resistive draw Example


This example shows the general process of installing an RGB panel driver, and displays a resistive touch NS2009 to createa rgb draw on the screen based on the LVGL library. . This example uses two kinds of **buffering mode** based on the number of frame buffers:

| Number of Frame Buffers | LVGL buffering mode | Way to avoid tear effect                                                                                    |
|-------------------------|---------------------|-------------------------------------------------------------------------------------------------------------|
| 1                       | Two buffers         | Extra synchronization mechanism is needed, e.g. using semaphore.                                            |
| 2                       | Full refresh        | There's no intersection between writing to an offline frame buffer and reading from an online frame buffer. |

This example also uses a sample kind of **hal layer** based on the config files utilized on the includes:

## How to use the example

### Hardware Required

* An ESP development board, which has RGB LCD peripheral supported and **Octal PSRAM** onboard
* A paralel RGB panel, 16 bit-width, with HSYNC, VSYNC and DE signal
* An USB cable for power supply and programming
