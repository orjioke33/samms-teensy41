#ifndef accel_h_
#define accel_h

#include <Arduino.h>

int8_t setup_accel(void);
void display_accel_data_rate(void);
void display_accel_range(void);
int8_t get_and_filter_raw_accel_data (void);

#endif