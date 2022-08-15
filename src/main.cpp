// https://forum.pjrc.com/threads/46150-Recording-Stereo-Audio-to-SD-Card?p=158682&viewfull=1#post158682
// https://forum.pjrc.com/threads/42562-MEMS-i2c-microphone-SPH0645LM4H-with-teensy-audio-library?p=160880&viewfull=1#post160880
// 16 bit 44.1khz

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
// i2s1 and i2s2 are already wired for Teensy boards
AudioInputI2S            i2s1;           //xy=168,145
AudioInputI2S2           i2s2;           //xy=168,145
AudioAnalyzeRMS          rmsLeft;
AudioAnalyzeRMS          rmsRight;
AudioRecordQueue         queue1;         //xy=360,62
AudioRecordQueue         queue2;         //xy=389,145
AudioConnection          patchCord1(i2s1, 1, queue1, 0); // Left Channel
AudioConnection          patchCord2(i2s2, 0, queue2, 0); // Right Channel
AudioConnection          patchCord3(i2s1, 1, rmsLeft, 0); // Left Channel
AudioConnection          patchCord4(i2s2, 0, rmsRight, 0); // Right Channel
// GUItool: end automatically generated code

// AudioControlSGTL5000     sgtl5000_1;     //xy=265,212


// which input on the audio shield will be used?
// const int myInput = AUDIO_INPUT_LINEIN;
//const int myInput = AUDIO_INPUT_MIC;


// Use these with the Teensy Audio Shield
// #define SDCARD_CS_PIN    10
// #define SDCARD_MOSI_PIN  7
// #define SDCARD_SCK_PIN   14

// Use these with the Teensy 3.5 & 3.6 SD card
#define SDCARD_CS_PIN    BUILTIN_SDCARD // 254?
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording, 2=playing

// The file where data is recorded
File frec;

void startRecording() {
  Serial.println("StartRecording");
  if (SD.exists("RECORD.RAW")) {
    SD.remove("RECORD.RAW");
  }
  frec = SD.open("RECORD.RAW", FILE_WRITE);
  if (frec) {
    Serial.println("File Open");
    queue1.begin();
    queue2.begin();
    mode = 1;
  }

}

bool actionTimer(time_t timeout_ms) {
  static time_t stamp = 0;
  if (millis() - stamp >= timeout_ms) {
    stamp = millis();
    return true;
  }
  return false;
}

void printRMSLeft() {
  if (rmsLeft.available()) {
    Serial.print("RMS Left: "); Serial.print(rmsLeft.read() * 100.0000, 6); Serial.print("\n");
  }
}

void printRMSRight() {
  if (rmsRight.available()) {
    Serial.print("RMS Right: "); Serial.print(rmsRight.read() * 100.0000, 6); Serial.print("\n");
  }
}

void printRMS() {
  printRMSLeft();
  printRMSRight();
}

// write all 512 bytes to the SD card   
void continueRecording() {
  if (queue1.available() >= 2 && queue2.available() >= 2) {
    byte buffer[512];
    byte bufferL[256];
    byte bufferR[256];
    memcpy(bufferL, queue1.readBuffer(), 256);
    memcpy(bufferR, queue2.readBuffer(), 256);
    queue1.freeBuffer();
    queue2.freeBuffer();
    int b = 0;
    for (int i = 0; i < 512; i += 4) {
      buffer[i] = bufferL[b];
      buffer[i + 1] = bufferL[b + 1];
      buffer[i + 2] = bufferR[b];
      buffer[i + 3] = bufferR[b + 1];
      b = b+2;
    }
    elapsedMicros usec = 0;
    frec.write(buffer, 512);  //256 or 512 (dudes code)
    if (actionTimer(2000)) {
      Serial.print("SD write, us=");
      Serial.println(usec);
    }
    if (actionTimer(500))
      printRMS();
  }
}

void stopRecording() {
  Serial.println("StopRecording");
  queue1.end();
  queue2.end();
  // flush buffer
  while (queue1.available() > 0 && queue2.available() > 0) {
    queue1.readBuffer();
    queue1.freeBuffer();
    queue2.readBuffer();
    queue2.freeBuffer();
  }
  frec.close(); // close file
  mode = 4;
}

void setup() {
  // record queue uses this memory to buffer incoming audio.
  AudioMemory(120); // 60

  // Enable the audio shield, select input, and enable output
   // sgtl5000_1.enable();
   // sgtl5000_1.inputSelect(myInput);
   // sgtl5000_1.volume(0.5);



  // Initialize the SD card
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here if no SD card, but print a message
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

  startRecording();
}


void loop() {
  if (millis() > 20000 && mode == 1) {
    stopRecording();
  }
  else {
    if (mode == 1) continueRecording();
  }
}