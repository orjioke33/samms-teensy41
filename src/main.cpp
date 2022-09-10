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

// Use these with the Teensy 3.5 & 3.6 SD card
#define SDCARD_CS_PIN    BUILTIN_SDCARD // 254?
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording, 2=playing
int indx = 0;

int16_t accelXBuffer[512];
int16_t accelYBuffer[512];
int16_t accelZBuffer[512];

// The file where data is recorded
File frecx;
File frecy;
File frecz;

void startRecording() {
  Serial.println("StartRecording");
 if (SD.exists("ACCELX__SilentSpeakQuiet2.txt")) {
    SD.remove("ACCELX__SilentSpeakQuiet2.txt");
  }
  frecx = SD.open("ACCELX__SilentSpeakQuiet2.txt", FILE_WRITE);
  if (frecx) {
    Serial.println("File Open X");
  }

  if (SD.exists("ACCELY_SilentSpeakQuiet2.txt")) {
    SD.remove("ACCELY__SilentSpeakQuiet2.txt");
  }
  frecy = SD.open("ACCELY__SilentSpeakQuiet2.txt", FILE_WRITE);
  if (frecy) {
    Serial.println("File Open Y");
  }

  if (SD.exists("ACCELZ__SilentSpeakQuiet2.txt")) {
    SD.remove("ACCELZ__SilentSpeakQuiet2.txt");
  }
  frecz = SD.open("ACCELZ__SilentSpeakQuiet2.txt", FILE_WRITE);
  if (frecz) {
    Serial.println("File Open Z");
  }

  mode = 1;

}

bool actionTimer(time_t timeout_ms) {
  static time_t stamp = 0;
  if (millis() - stamp >= timeout_ms) {
    stamp = millis();
    return true;
  }
  return false;
}

// write all 512 bytes to the SD card   
void continueRecording() {
  sensors_event_t event;

  // Save x y z data into buffers
  if(sysConfig.accel.getEvent(&event)) {
    sysConfig.accel.getXYZ(accelXBuffer[indx], accelYBuffer[indx], accelZBuffer[indx]);
    if (indx > 511) {
      indx = 0;
      // Write to file
      elapsedMicros usec = 0;
      Serial.print("SD write, us = ");
      Serial.println(usec);
      frecx.write(accelXBuffer, sizeof(int16_t) * 512);
      frecy.write(accelYBuffer, sizeof(int16_t) * 512);
      frecz.write(accelZBuffer, sizeof(int16_t) * 512);
      Serial.println("Wrote x y z.");
    } else {
      indx++;
    }
  }
}

void stopRecording() {
  Serial.println("StopRecording");
  frecx.close(); // close file
  frecy.close();
  frecz.close();
  mode = 4;
}

void setup() {
  // record queue uses this memory to buffer incoming audio.
  AudioMemory(120); // 60

  delay(5000);
  while (samms_setup() != ERR_SAMMS_OK) {
      delay(5000); // Check every 5 seconds.
  }

  Serial.println(5);
  delay(1000);
  Serial.println(4);
  delay(1000);
  Serial.println(3);
  delay(1000);
  Serial.println(2);
  delay(1000);
  Serial.println(1);
  delay(1000);

  startRecording();
}


void loop() {
  if (millis() > 40000 && mode == 1) {
    stopRecording();
  }
  else {
    if (mode == 1) continueRecording();
  }
}