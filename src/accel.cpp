#include <Wire.h>
#include <SerialFlash.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

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
static void do_accel_median_filter (int16_t * buf) {
  // value 1, 2, 3
  int16_t m1, m2, m3;

  for (int k = 0; k < DEFAULT_ACCEL_BUFFER_SIZE; k++) {
    int k1 = max(1, k - 1);
    int k2 = min(DEFAULT_ACCEL_BUFFER_SIZE - 2, k + 1);
    m1 = buf[k];
    m2 = buf[k1];
    m3 = buf[k2];

    // Replace the curr value with the median of itself
    // and its neighbors
    if ((m1 <= m2 && m1 >= m3) || (m1 >= m2 && m1 <= m3)) {
      buf[k] = m1;
    } else if ((m2 <= m1 && m2 >= m3) || (m2 >= m1 && m2 <= m3)) {
      buf[k] = m2;
    } else if ((m3 <= m2 && m3 >= m1) || (m3 >= m2 && m3 <= m1)) {
      buf[k] = m3;
    }
  }
}

// Save x y z data into buffers
// Returns 0 if data buffer isn't full
// Returns 1 if data buffers are full
// Returns other value for an error
int8_t get_and_filter_raw_accel_data (void) {
  int8_t buffersFull = 0;
  static int indx = 0;

  if (sysConfig.accel.getEvent(&sysData.accelData.event)) {
    sysConfig.accel.getXYZ(sysData.accelData.xRaw[indx], sysData.accelData.yRaw[indx], sysData.accelData.zRaw[indx]);
    if (indx > 511) {
      // Buffers are full, so we can filter
      do_accel_median_filter(sysData.accelData.xRaw);
      do_accel_median_filter(sysData.accelData.yRaw);
      do_accel_median_filter(sysData.accelData.zRaw);
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