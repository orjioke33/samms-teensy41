#ifndef nlms_h_
#define nlms_h_

#include <Arduino.h>
#include "arm_math.h"

void do_nlms(double *x, double *d, double *dhat, double *e, double *w, double mu, int N, int xlen);
void apply_window_to_fft_buffer(void *buffer, const void *window);
void copy_to_fft_buffer(void *destination, const void *source);
void apply_fft(uint16_t* output, int16_t* buffer, arm_cfft_radix4_instance_q15 fft_inst);

#endif