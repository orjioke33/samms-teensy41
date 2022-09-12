#ifndef accel_h_
#define accel_h

#include <Arduino.h>

int8_t setup_accel(void);
void display_accel_data_rate(void);
void display_accel_range(void);
void accel_thread(void);

#endif