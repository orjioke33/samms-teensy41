#include "nlms.h"
#include "arm_math.h"
#include "sqrt_integer.h"
#include "utility/dspinst.h"

void do_nlms(double *x, double *d, double *dhat, double *e, double *w, double mu, int N, int xlen)
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

void apply_fft(float32_t* output, float32_t* buffer, arm_cfft_radix4_instance_f32 fft_inst) {
  //wrong arm cfft function i think
    arm_cfft_radix4_f32(&fft_inst, buffer);
    for (int i=0; i < 512; i++) {
        uint32_t tmp = *((uint32_t *)buffer + i); // real & imag
        uint32_t magsq = multiply_16tx16t_add_16bx16b(tmp, tmp);
        output[i] = sqrt_uint32_approx(magsq) * (1.0f / 16384.0f);
    }
}

void copy_to_fft_buffer(void *destination, const void *source)
{
	const float32_t *src = (const float32_t *)source;
	float32_t *dst = (float32_t *)destination;

	for (int i=0; i < 1024; i+=2) {
		dst[i] = (float32_t) src[i];
	}
}

void apply_window_to_fft_buffer(void *buffer, const void *window)
{
	int16_t *buf = (int16_t *)buffer;
	const int16_t *win = (int16_t *)window;;

	for (int i=0; i < 1024; i++) {
		int32_t val = *buf * *win++;
		//*buf = signed_saturate_rshift(val, 16, 15);
		*buf = val >> 15;
		buf += 2;
	}

}