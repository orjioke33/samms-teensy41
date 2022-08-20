/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Pete (El Supremo)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Arduino.h>
#include "filter_nlms.h"

#define BUFFER_SIZE 512
#define FREQ_BANDS_N 128
#define MU  0.1f

// Saves the difference and summation of
// left and right buffers
static void get_diff_and_sum (double * diff, double * sum, int16_t * left, int16_t * right) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        diff[i] = left[i] - right[i];
        sum[i] = left[i] + right[i];
    }
}

static void donlms(double *x, double *d, double *dhat, double *e, double *w, double mu, int N, int xlen)
{

#ifdef NOTDEFINED
  // lms(x,d,dhat,e,w,xlen,N,mu)
  // double    *x;               /* pointer to input data buffer */
  // double    *d;               /* pointer to desired signal buffer */
  // double    *w;               /* weight vector */
  // double    *dhat;            /* estimate of d produced by filter */
  // double    *e;               /* error */
  // int        xlen;            /* length of input data buffer */
  // int        N;               /* filter length */
  // double     mu;              /* mu = 2 x's the conventional mu */
#endif

   /* Note: the input data is in a buffer that is actually L+N long */
   /* so there will be enough data to fill the filter taps as well. */
   
   register double s;       /* summer used in filter */
   register int j,i;        /* loop counters */
   double *x1;              /* temporary pointer to input data */
   double *x2;              /* temporary pointer to input data */
   double *wn,*wn0,*ee;     /* temporary pointer to filter weights */
   double Em = 1e-20;
   

/****************************************
 ******** Matlab equivalent code ********
   for n = N:length(x),
     % produce filtered output sample
     dhat(n) = w * x(n:-1:n-(N-1))';
     % update the filter coefficients
     e(n) = d(n) - dhat(n);
     w = w+2*mu*x(n:-1:n-(N-1))*e(n);
   end
*****************************************/

   for(i=0; i<N; i++) {
     Em += x[i] * x[i];
   }

   for(i=0; i< xlen; i++)
   {
     for(wn = w, x1=x+i, j=0,s=0.0; j<N; j++)
       s += *(wn++) * *(x1--);
     x1++;
     Em += x[i] * x[i] - *x1 * *x1;
     e[i] = d[i] - s;
     dhat[i] = s;
	 // the following multiplication can be made more efficient by using frexpf() to extract the exponent of Em, then use ldexpf() to multiply mu*e[i] by 2^(-exp).  The approximation is fine...
     s = mu * e[i] / Em;
	 // The following weight update is for a regular nLMS adaptive filter.  For our system we will need to ensure that the weights stay below 2 in magnitude.  If they get bigger than that, just cap them at that value.
     for(j=0; j<N; j++)
       w[j] += s * x[i-j];
   }
}

void AudioFilterNLMS::update(void)
{
    static int updateNum = 0;
    static int buffInd = 0;
    double diff[BUFFER_SIZE] = {0}, sum[BUFFER_SIZE] = {0}, noiseOut[BUFFER_SIZE] = {0};
    audio_block_t *micLeft = NULL, *micRight = NULL, *monoOut = NULL;

    // grab 128 samples from left and right microphones
    micLeft = receiveReadOnly(0);
    micRight = receiveReadOnly(1);

    if (micLeft == NULL || micRight == NULL) { return; }

    // copy the mic data into buffers for use later
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        left[buffInd] = micLeft->data[i];
        right[buffInd] = micRight->data[i];
    }

    updateNum++;
    buffInd = updateNum * AUDIO_BLOCK_SAMPLES;

    // filter once we hit 512 samples
    if (updateNum == 4) {
        Serial.println("WE HIT 4!!!!!!!!");
        updateNum = 0;
        buffInd = 0;
        get_diff_and_sum(diff, sum, left, right);
        donlms(diff, sum, noiseOut, (double *) monoOut->data, aWeight, MU, FREQ_BANDS_N, BUFFER_SIZE);
        transmit(monoOut, 0);
    }

    // release or 'free' all receiveReadOnly blocks
    release(micLeft);
    release(micRight);
}