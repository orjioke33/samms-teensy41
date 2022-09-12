// https://forum.pjrc.com/threads/46150-Recording-Stereo-Audio-to-SD-Card?p=158682&viewfull=1#post158682
// https://forum.pjrc.com/threads/42562-MEMS-i2c-microphone-SPH0645LM4H-with-teensy-audio-library?p=160880&viewfull=1#post160880
// 16 bit 44.1khz

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <TeensyThreads.h>

#include "board.h"
#include "accel.h"
#include "file_system.h"
#include "nlms.h"
#include "arm_math.h"
#include "sqrt_integer.h"

AudioConnection          patchCord1(sysConfig.mic.i2sL, 0, sysConfig.mic.queue1, 0); // Left Channel
AudioConnection          patchCord2(sysConfig.mic.i2sR, 0, sysConfig.mic.queue2, 0); // Right Channel

void setup() {
  // record queue uses this memory
  // to buffer incoming audio
  AudioMemory(120);

  // setup driver, motor, sd card
  // and accelerometer
  delay(8000);
  Serial.println("Setting up SAMMS...");
  while (samms_setup() != ERR_SAMMS_OK) {
      delay(5000); // Check every 5 seconds.
  }

  Serial.println("SAMMS Setup succeeded. Starting mic sampling...");
  sysConfig.mic.queue1.begin();
  sysConfig.mic.queue2.begin();
  delay(5000);
  // Thread function, thread arguments, stack size in bytes
  threads.addThread(accel_thread, 0, 8192);
  threads.addThread(mic_filter_thread, 0, 8192);
  Serial.println("Starting accel sampling...");
}


void loop() {
  static int64_t x = 0;
  if (millis() - x > 10000) {
    x += 10000;
    sysData.uptimeSeconds = millis() / 1000.000;
    Serial.print("MAIN: 10s passed: ");
    Serial.println(sysData.uptimeSeconds);
  }
}