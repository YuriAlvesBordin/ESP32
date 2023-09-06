#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <touch_ns2009.h>
#include <driver/i2c.h>

#define NS2009_ADDR 0x48

#define NS2009_LOW_POWER_READ_X 0xc0
#define NS2009_LOW_POWER_READ_Y 0xd0
#define NS2009_LOW_POWER_READ_Z1 0xe0

#define SCREEN_X_PIXEL 480
#define SCREEN_Y_PIXEL 272

// Define screen size in inches
#define SCREEN_WIDTH_INCHES 4.3
#define SCREEN_HEIGHT_INCHES 2.5

static void ns2009_recv(const uint8_t *send_buf, size_t send_buf_len, uint8_t *receive_buf,size_t receive_buf_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t i2c_ret;
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (NS2009_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, send_buf, send_buf_len, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (NS2009_ADDR << 1) | I2C_MASTER_READ, true);
    if (receive_buf_len > 1) {
        i2c_ret = i2c_master_read(cmd, receive_buf, receive_buf_len - 1, I2C_MASTER_ACK);
        if (i2c_ret != ESP_OK) {
            printf("I2C Read Error: %s\n", esp_err_to_name(i2c_ret));
        }
    }
    i2c_ret = i2c_master_read_byte(cmd, receive_buf + receive_buf_len - 1, I2C_MASTER_NACK);
    if (i2c_ret != ESP_OK) {
        printf("I2C Read Error: %s\n", esp_err_to_name(i2c_ret));
    }
    i2c_master_stop(cmd);
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    if (i2c_ret != ESP_OK) {
        printf("I2C Command Error: %s\n", esp_err_to_name(i2c_ret));
    }
    i2c_cmd_link_delete(cmd);
}

static unsigned int ns2009_read(uint8_t cmd)
{
    uint8_t buf[2];
    ns2009_recv(&cmd, 1, buf, 2);
    return (buf[0] << 4) | (buf[1] >> 4);
}

int ns2009_get_press()
{
    return ns2009_read(NS2009_LOW_POWER_READ_Z1);

}

int ns2009_pos(int pos[2])
{
    int press = ns2009_read(NS2009_LOW_POWER_READ_Z1);

    int x_raw, y_raw = 0;

    x_raw = ns2009_read(NS2009_LOW_POWER_READ_X);
    y_raw = ns2009_read(NS2009_LOW_POWER_READ_Y);

    // Calculate scaling factors based on screen size in inches
    float x_scale = SCREEN_X_PIXEL / (SCREEN_WIDTH_INCHES); // Convert inches to millimeters
    float y_scale = SCREEN_Y_PIXEL / (SCREEN_HEIGHT_INCHES); // Convert inches to millimeters

    // Calculate adjusted position with border offset
    int x_offset = (x_raw - 2048) / x_scale;
    int y_offset = (y_raw - 2048) / y_scale;

    pos[0] = -(SCREEN_X_PIXEL * (x_raw - 4096) / 4096) - x_offset;
    pos[1] = -(SCREEN_Y_PIXEL * (y_raw - 4096) / 4096) - y_offset;

    // Calculate a larger pressure offset based on the X position
    int x_pressure_offset = (pos[0] - SCREEN_X_PIXEL / 2) * 3.5; // Increase the factor to apply a larger offset

    // Apply the pressure offset
    press += x_pressure_offset;

    return press;
}
