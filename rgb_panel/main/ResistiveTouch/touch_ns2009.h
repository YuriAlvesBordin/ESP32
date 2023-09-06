#ifndef NS2009_TOUCH_H
#define NS2009_TOUCH_H

// Function prototypes
static void ns2009_recv(const uint8_t *send_buf, size_t send_buf_len, uint8_t *receive_buf,
                size_t receive_buf_len);
static unsigned int ns2009_read(uint8_t cmd);
int ns2009_get_press();
int ns2009_pos(int pos[2]);

#endif /* NS2009_TOUCH_H */
