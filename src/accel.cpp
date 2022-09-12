#include <Wire.h>
#include <SerialFlash.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include <TeensyThreads.h>

#include "board.h"
#include "accel.h"

int8_t setup_accel(void) {
    int8_t err = ERR_SAMMS_OK;

    // Detect the accelerometer
    if (!sysConfig.accel.begin()) {
        err = ERR_ACCEL_BEGIN_FAIL;
        Serial.println("Accel failed to begin.");
        goto quit;
    }

    // Set specifications
    sysConfig.accel.setRange(ADXL343_RANGE_4_G);
    sysConfig.accel.setDataRate(ADXL3XX_DATARATE_800_HZ);
    display_accel_data_rate();
    display_accel_range();

quit:
    return err;
}

void display_accel_data_rate(void)
{
  Serial.print  ("Data Rate:    ");

  switch(sysConfig.accel.getDataRate())
  {
    case ADXL343_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

void display_accel_range(void)
{
  Serial.print  ("Range:         +/- ");

  switch(sysConfig.accel.getRange())
  {
    case ADXL343_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL343_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL343_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL343_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}

// TODO: do all 3 axes in one for loop
// Maybe copy these values to new buffers so we can
// still sample while doing filtering / testing for
// speech
static void do_accel_median_filter (int16_t * xBuf, int16_t * yBuf, int16_t * zBuf) {
  // value 1, 2, 3
  int16_t x1, x2, x3;
  int16_t y1, y2, y3;
  int16_t z1, z2, z3;

  for (int k = 0; k < DEFAULT_ACCEL_BUFFER_SIZE; k++) {
    int k1 = max(1, k - 1);
    int k2 = min(DEFAULT_ACCEL_BUFFER_SIZE - 2, k + 1);
    x1 = xBuf[k]; x2 = xBuf[k1]; x3 = xBuf[k2];
    y1 = yBuf[k]; y2 = yBuf[k1]; y3 = yBuf[k2];
    z1 = zBuf[k]; z2 = zBuf[k1]; z3 = zBuf[k2];

    // Replace the curr value with the median of itself
    // and its neighbors
    // X axis
    if ((x1 <= x2 && x1 >= x3) || (x1 >= x2 && x1 <= x3)) {
      xBuf[k] = x1;
    } else if ((x2 <= x1 && x2 >= x3) || (x2 >= x1 && x2 <= x3)) {
      xBuf[k] = x2;
    } else if ((x3 <= x2 && x3 >= x1) || (x3 >= x2 && x3 <= x1)) {
      xBuf[k] = x3;
    }

    // Y axis
    if ((y1 <= y2 && y1 >= y3) || (y1 >= y2 && y1 <= y3)) {
      yBuf[k] = y1;
    } else if ((y2 <= y1 && y2 >= y3) || (y2 >= y1 && y2 <= y3)) {
      yBuf[k] = y2;
    } else if ((y3 <= y2 && y3 >= y1) || (y3 >= y2 && y3 <= y1)) {
      yBuf[k] = y3;
    }

    // Z axis
    if ((z1 <= z2 && z1 >= z3) || (z1 >= z2 && z1 <= z3)) {
      zBuf[k] = z1;
    } else if ((z2 <= z1 && z2 >= z3) || (z2 >= z1 && z2 <= z3)) {
      zBuf[k] = z2;
    } else if ((z3 <= z2 && z3 >= z1) || (z3 >= z2 && z3 <= z1)) {
      zBuf[k] = z3;
    }
  }
}

//TODO: Test
//ERROR: oldVal will use the previous value of
// a different axis
static void do_high_pass_filter (int16_t * xBuf, int16_t * yBuf, int16_t * zBuf) {
  static int16_t xOld = 0;
  static int16_t yOld = 0;
  static int16_t zOld = 0;

  xBuf[0] = xBuf[0] - xOld;
  yBuf[0] = yBuf[0] - yOld;
  zBuf[0] = zBuf[0] - zOld;

  for (int i = 1; i < DEFAULT_ACCEL_BUFFER_SIZE; i++) {
    xBuf[i] = xBuf[i] - xBuf[i - 1];
    yBuf[i] = yBuf[i] - yBuf[i - 1];
    zBuf[i] = zBuf[i] - zBuf[i - 1];
  }

  xOld = xBuf[DEFAULT_ACCEL_BUFFER_SIZE - 1];
  yOld = yBuf[DEFAULT_ACCEL_BUFFER_SIZE - 1];
  zOld = zBuf[DEFAULT_ACCEL_BUFFER_SIZE - 1];
}

// Save x y z data into buffers
// Returns 0 if data buffer isn't full
// Returns 1 if data buffers are full
// Returns other value for an error
static int8_t get_and_filter_raw_accel_data (void) {
  int8_t buffersFull = 0;
  static int indx = 0;

  if (sysConfig.accel.getEvent(&sysData.accelData.event)) {
    sysConfig.accel.getXYZ(sysData.accelData.xRaw[indx], sysData.accelData.yRaw[indx], sysData.accelData.zRaw[indx]);
    // If buffers are full, then we filter
    if (indx > DEFAULT_ACCEL_BUFFER_SIZE - 1) {
      // Write it
      if (sysData.uptimeSeconds < 34) {
        sysData.files.faccelx.write(sysData.accelData.xRaw, sizeof(sysData.accelData.xRaw));
        sysData.files.faccely.write(sysData.accelData.yRaw, sizeof(sysData.accelData.yRaw));
        sysData.files.faccelz.write(sysData.accelData.zRaw, sizeof(sysData.accelData.zRaw));
        Serial.println("Wrote x y z accel files");
      }
      // Median filters
      do_accel_median_filter(sysData.accelData.xRaw, sysData.accelData.yRaw, sysData.accelData.zRaw);
      // High pass filters
      do_high_pass_filter(sysData.accelData.xRaw, sysData.accelData.yRaw, sysData.accelData.zRaw);
      // The z axis detects speech. Let's copy it so we can continue sampling
      memcpy(sysData.accelData.zCopy, sysData.accelData.zRaw, sizeof(sysData.accelData.zRaw));
      // Reset
      indx = 0;
      buffersFull = 1;
    } else {
      indx++;
      buffersFull = 0;
    }
  }

  return buffersFull;
}

static void get_accel_z_average (float * speechAvg, float * bgAvg) {
  // Square the z data and get the averages
  static float oldZSpeech = 0, oldZBg = 0;
  float zSpeechSum = 0, zBgSum = 0;
  float zSq[DEFAULT_ACCEL_BUFFER_SIZE] = {0};
  float zSpeech[DEFAULT_ACCEL_BUFFER_SIZE] = {0};
  float zBg[DEFAULT_ACCEL_BUFFER_SIZE] = {0};

  zSq[0] = sysData.accelData.zCopy[0] * sysData.accelData.zCopy[0];
  zSpeech[0] = 0.02 * zSq[0] + 0.98 * oldZSpeech;
  zBg[0] = 0.001 * zSq[0] + 0.999 * oldZBg;
  zSpeechSum = zSpeech[0]; zBgSum = zBg[0];

  for (int i = 1; i < DEFAULT_ACCEL_BUFFER_SIZE; i++) {
    zSq[i] = sysData.accelData.zCopy[i] * sysData.accelData.zCopy[i];
    zSpeech[i] = 0.02 * zSq[i] + 0.98 * zSpeech[i - 1];
    zBg[i] = 0.001 * zSq[i] + 0.999 * zBg[i - 1];

    zSpeechSum += zSpeech[i];
    zBgSum += zBg[i];
  }

  oldZSpeech = zSpeech[DEFAULT_ACCEL_BUFFER_SIZE - 1];
  oldZBg = zBg[DEFAULT_ACCEL_BUFFER_SIZE - 1];

  *speechAvg = zSpeechSum / DEFAULT_ACCEL_BUFFER_SIZE;
  *bgAvg = zBgSum / DEFAULT_ACCEL_BUFFER_SIZE;
}

static bool is_accel_speech_detected (void) {
  // If the raw data has been filtered, then
  // we can start detect speech
  bool isSpeaking = false;
  float speechAvg = 0, bgAvg = 0;
  if (get_and_filter_raw_accel_data() == 1) {
    get_accel_z_average(&speechAvg, &bgAvg);
    if (speechAvg > 2 * bgAvg) {
      isSpeaking = true;
    } else {
      isSpeaking = false;
    }
  }

  return isSpeaking;
}

void accel_thread (void) {
  bool fileClosed = false;
  while(1) {
    static int64_t x = 0;
    sysStatus.isSpeechDetected = is_accel_speech_detected();
    if (sysStatus.isSpeechDetected) {
      Serial.println("Speech detected!!!!");
    }
    threads.yield();
    if (millis() - x > 2000) {
      Serial.println("ACCEL: 2s passed.");
      x += 2000;
    }
    if (sysData.uptimeSeconds >= 32 && !fileClosed) {
      Serial.println("closed x files");
      sysData.files.faccelx.close();
      Serial.println("closed y files");
      sysData.files.faccely.close();
      Serial.println("closed z files");
      sysData.files.faccelz.close();
      fileClosed = true;
    }
  }

  // We should never reach the end of the thread
  Serial.println("Accel thread ERROR!!!");
}