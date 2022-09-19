#include <Arduino.h>
#include <Audio.h>
#include <TeensyThreads.h>

#include "nlms.h"
#include "board.h"
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

//   for(i=xlen-N; i<xlen; i++) {
//     Em += x[i] * x[i];
//   }
  //filter (The second half of the array has the new values to be filtered)
   for(i=0; i < xlen; i++)
   {
     for(wn = w, x1=x+i+xlen, j=0,s=0.0, Em=1e-6; j<N; j++){
       Em += *x1 * *x1;
       s += *(wn++) * *(x1--);
     }
     
 //    Em += x[i+xlen] * x[i+xlen] - *x1 * *x1;
     e[i] = d[i+xlen] - s;
     dhat[i] = s;
	 // the following multiplication can be made more efficient by using frexpf() to extract the exponent of Em, then use ldexpf() to multiply mu*e[i] by 2^(-exp).  The approximation is fine...
     s = mu * e[i] / Em;

	 // The following weight update is for a regular nLMS adaptive filter.  For our system we will need to ensure that the weights stay below 2 in magnitude.  If they get bigger than that, just cap them at that value.
     for(j=0; j<N; j++){
        w[j] += s * x[i-j+xlen];
        w[j] *= .999999;
        if(w[j]>2.0)
          w[j]=2.0;
        if(w[j]<-2.0)
          w[j]=-2.0;
     }
       
   }
}

void apply_fft(float32_t* output, float32_t* buffer, arm_cfft_radix4_instance_f32 fft_inst) {
  //wrong arm cfft function i think
    arm_cfft_radix4_f32(&fft_inst, buffer);
    for (int i=0; i < 512; i++) {
        float32_t tmp = *((float32_t *)buffer + i); // real & imag
        float32_t magsq = multiply_16tx16t_add_16bx16b(tmp, tmp);
        output[i] = sqrt_uint32_approx(magsq) * (1.0f / 16384.0f);
    }
}

void copy_to_fft_buffer(void *destination, const void *source)
{
	const double *src = (const double *)source;
	float32_t *dst = (float32_t *)destination;

	for (int i=0; i < 512; i++) {
		dst[2*i] = (float32_t) src[i];
    dst[2*i+1] = 0.0;
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

void write_raw_mic_data_to_sd (float timeRecording) {

    short fullBuffer[256] = {0};
    static bool closed = false;
    int b = 0;
    for (int i = 0; i < 256; i+=2) {
      fullBuffer[i] = sysConfig.micFilter.micLeftBuffer[b];
      fullBuffer[i+1] = sysConfig.micFilter.micRightBuffer[b];
      b++;
    }

    elapsedMicros usec = 0;
    if (!closed && sysData.uptimeSeconds < timeRecording && sysData.files.fmicraw) {
      sysData.files.fmicraw.write(fullBuffer, 512);  //256 or 512 (dudes code)
      Serial.print("write raw data. usec = "); Serial.println(usec);
    }

    if (sysData.files.fmicraw && sysData.uptimeSeconds >= timeRecording) {
      Serial.println("Closing raw files");
      sysData.files.fmicraw.close();
      closed = true;
    }
  
}

void write_nlms_out_to_sd_card (float timeRecording) {
    static bool closed = false;

    elapsedMicros usec = 0;
    if (!closed && sysData.uptimeSeconds < timeRecording && sysData.files.fnlms) {
      sysData.files.fnlms.write(sysConfig.micFilter.nlmsOut, sizeof(double) * 128);  //256 or 512 (dudes code)
      Serial.print("write raw data. usec = "); Serial.println(usec);
    }

    if (sysData.files.fnlms && sysData.uptimeSeconds >= timeRecording) {
      Serial.println("Closing raw files");
      sysData.files.fnlms.close();
      closed = true;
    }
}

void mic_filter_thread (void) {
  double yL_old = 0;
  double yR_old = 0;
  double micL_old = 0;
  double micR_old = 0;
  int8_t quietCounter = 0;
  float32_t v[512] = {0};
  bool closed = false;

  //bins [7 42] should have weights 1 for BP filtering
  for(int i=0; i<512; i++) {
    if(i > 42){
      sysConfig.micFilter.bp_weight[i] = 0.0;
    } else if(i > 6) {
      sysConfig.micFilter.bp_weight[i] = 1.0;
    } else {
      sysConfig.micFilter.bp_weight[i] = 0.0;
    }
  }

  while (1) {
    if (sysConfig.mic.queue1.available() >= 2 && sysConfig.mic.queue2.available() >= 2) {
      double micNoise[BUFFER_SIZE_MIC];

      for (int i = 0; i < DELAYOFFSET; i++)
        sysConfig.micFilter.micLeftBuffer[i] = sysConfig.micFilter.micLeftBuffer[i+BUFFER_SIZE_MIC];

      // If >= 1024 bytes of mic data is available, save in a buffer
      memcpy(sysConfig.micFilter.micLeftBuffer+DELAYOFFSET, sysConfig.mic.queue1.readBuffer(), BUFFER_SIZE_MIC*2);
      memcpy(sysConfig.micFilter.micRightBuffer, sysConfig.mic.queue2.readBuffer(), BUFFER_SIZE_MIC*2);
      sysConfig.mic.queue1.freeBuffer();
      sysConfig.mic.queue2.freeBuffer();

      // write_raw_mic_data_to_sd(40);

      //remove DC offset
      sysConfig.micFilter.yL[0] = sysConfig.micFilter.micLeftBuffer[0] -
                                  micL_old - 0.95*yL_old;
      sysConfig.micFilter.yR[0] = sysConfig.micFilter.micRightBuffer[0] -
                                  micR_old - 0.95*yR_old;

      for(int n=1; n < BUFFER_SIZE_MIC; n++){
        sysConfig.micFilter.yL[n] = sysConfig.micFilter.micLeftBuffer[n] - 
                                    sysConfig.micFilter.micLeftBuffer[n-1] - 0.95*sysConfig.micFilter.yL[n-1];
        sysConfig.micFilter.yR[n] = sysConfig.micFilter.micRightBuffer[n] - 
                                    sysConfig.micFilter.micRightBuffer[n-1] - 0.95*sysConfig.micFilter.yR[n-1];
      }

      yL_old = sysConfig.micFilter.yL[BUFFER_SIZE_MIC-1];
      yR_old = sysConfig.micFilter.yR[BUFFER_SIZE_MIC-1];
      micL_old = sysConfig.micFilter.micLeftBuffer[BUFFER_SIZE_MIC-1]; 
      micR_old = sysConfig.micFilter.micRightBuffer[BUFFER_SIZE_MIC-1]; 

      // copy second half to first half
      for (int i = 0; i < BUFFER_SIZE_MIC; i++) {
        sysConfig.micFilter.micDiff[i] = sysConfig.micFilter.micDiff[i + BUFFER_SIZE_MIC];
        sysConfig.micFilter.micSum[i] = sysConfig.micFilter.micSum[i + BUFFER_SIZE_MIC];
      }

      //store new data in second half
      for (int i = 0; i < BUFFER_SIZE_MIC; i++) {
        sysConfig.micFilter.micDiff[i+BUFFER_SIZE_MIC] = 0.78*sysConfig.micFilter.yL[i] - 
          sysConfig.micFilter.yR[i];
        sysConfig.micFilter.micSum[i+BUFFER_SIZE_MIC] = 0.78*sysConfig.micFilter.yL[i] + 
          sysConfig.micFilter.yR[i];
      }


      // Filter background noise
      do_nlms(sysConfig.micFilter.micDiff, sysConfig.micFilter.micSum, micNoise, sysConfig.micFilter.nlmsOut,
              sysConfig.micFilter.nlms_weights, 0.1, 100, BUFFER_SIZE_MIC);

      // write_nlms_out_to_sd_card(40);

      // Do fft
      copy_to_fft_buffer(sysConfig.fftConfig.buffer, sysConfig.micFilter.nlmsOut);
      arm_cfft_f32(&sysConfig.fftConfig.fft_inst, sysConfig.fftConfig.buffer, 0, 1);//no bit reverse
      arm_cmplx_mag_squared_f32(sysConfig.fftConfig.buffer, sysConfig.fftConfig.output, 512);

      //BP and aweight filtering
      sysData.micEnergyData.magnitude = 0;
      sysData.micEnergyData.diffMagSq = 0;
      sysData.dBStats.curr = 0;
      
      for (int i=0; i<512; i++) {
        v[i] = sysConfig.fftConfig.output[i] * sysConfig.micFilter.bp_weight[i] * 1.0f/16384.0f; //* aWeight[i] * bp_weight[i] * 1.0f/16384.0f; //* 1.0f/262144.0f;//* 1/(512^2)
        sysData.micEnergyData.magnitude = sysData.micEnergyData.magnitude + v[i]; // + sq(v[i]); already squared
        sysData.micEnergyData.diffMagSq = sysData.micEnergyData.diffMagSq + sq(abs(sysConfig.micFilter.micDiff[i+BUFFER_SIZE_MIC]));
      }

      //get spl and buzz?

      sysData.dBStats.curr = log10f(sysData.micEnergyData.magnitude) * 10  + 80.05;
      sysData.dBStats.sum += sysData.dBStats.curr;
      sysData.dBStats.count++;
      sysData.dBStats.avg = sysData.dBStats.sum / sysData.dBStats.count;
      
      // Check for buzz every 96 dB samples
      if (sysData.dBStats.count >= 32) {
      //nlmsOut
      //micDiff second half
        // if(sysData.micEnergyData.diffMagSq > (sysData.micEnergyData.noiseConstant * sysData.micEnergyData.magnitude)) {
        //   if (sysData.dBStats.avg > sysConfig.splUserConfig.splLowerdBA && sysData.dBStats.avg < sysConfig.splUserConfig.splUpperdBA && !sysStatus.isMotorOn) {
        //       // Serial.println(sysData.dBStats.avg,2); // f[23] = 1kHz, f[82] = 3.5kHz, f[252] = 12kHz
        //       //buzzOn(); delay(250); buzzOff();
        //   }
        // }

        if (sysData.dBStats.avg < sysConfig.splUserConfig.splLowerdBA) {
          quietCounter++;
          if (quietCounter >= 6) {
            // Around 3 seconds of the wearer speaking softly. Let's buzz.
            // samms_toggle_buzz(true);
            quietCounter = 0;
          } else {
            // samms_toggle_buzz(false);
          }
        } else {
          quietCounter = 0; // reset
        }
        Serial.print("dB: "); Serial.println(sysData.dBStats.avg);
        sysData.dBStats.sum = 0;
        sysData.dBStats.count = 0;
        sysData.dBStats.avg = 0;
      }
    }
  }
  Serial.println("MIC FILTER THREAD ERROR!!!!");
}