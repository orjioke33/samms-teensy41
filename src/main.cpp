// https://forum.pjrc.com/threads/46150-Recording-Stereo-Audio-to-SD-Card?p=158682&viewfull=1#post158682
// https://forum.pjrc.com/threads/42562-MEMS-i2c-microphone-SPH0645LM4H-with-teensy-audio-library?p=160880&viewfull=1#post160880
// 16 bit 44.1khz

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include "board.h"
#include "accel.h"
#include "file_system.h"
#include "nlms.h"
#include "arm_math.h"
#include "sqrt_integer.h"

// i2s1 and i2s2 are already wired for Teensy boards
AudioInputI2S            i2sL;           //xy=168,145
AudioInputI2S2           i2sR;           //xy=168,145
AudioRecordQueue         queue1;         //xy=360,62
AudioRecordQueue         queue2;         //xy=389,145
AudioConnection          patchCord1(i2sL, 0, queue1, 0); // Left Channel
AudioConnection          patchCord2(i2sR, 0, queue2, 0); // Right Channel

bool actionTimer(time_t timeout_ms) {
  static time_t stamp = 0;
  if (millis() - stamp >= timeout_ms) {
    stamp = millis();
    return true;
  }
  return false;
}

void setup() {
  // record queue uses this memory
  // to buffer incoming audio
  AudioMemory(120);

  // setup driver, motor, sd card
  // and accelerometer
  while (samms_setup() != ERR_SAMMS_OK) {
      delay(5000); // Check every 5 seconds.
  }
}


void loop() {
  get_and_filter_raw_accel_data();
}