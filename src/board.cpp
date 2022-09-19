#include <Audio.h>
#include "board.h"
#include "file_system.h"
#include "accel.h"

// Initialize system values
// sysConfig
samms_sys_config_t sysConfig = {
    .splUserConfig = {
        "spl.txt", // Cannot use .filename =, have to use C89 standard
        .splLowerdBA = -1,
        .splUpperdBA = -1,
    },
    .accel = Adafruit_ADXL343(0),
    .micFilter = {
        .yL = {0},
        .yR = {0},
        .micSum = {0},
        .micDiff = {0},
        .nlms_weights = {0},
        .micLeftBuffer = {0},
        .micRightBuffer = {0},
    },
    .fftConfig = {
        .buffer = {0},
        .cmplx_mag = {0},
    }
};

// sysData
samms_sys_data_t sysData = {
    .dBStats = {
        .curr = 0,
        .sum = 0,
        .avg = 0,
        .count = 0,
        .currSpeaker = 0,
        .sumSpeaker = 0,
        .avgSpeaker = 0,
        .countSpeaker = 0,
    },
    .micEnergyData = {
        .magnitude = 0,
        .diffMagSq = 0,
        .noiseConstant = 0.5,
    },
    .uptimeSeconds = 0,
    .buzzOnTimeStamp_ms = 0,
};

// sysStatus
samms_sys_status_t sysStatus = {
    .isMotorOn = false,
    .isSpeechDetected = false,
    .runTest = RUN_AVERAGE_DB_TEST,
};

// Setup the SD card, accelerometer, haptic driver & motor
int8_t samms_setup(void) {
    int8_t err = ERR_SAMMS_OK;

    // Setup haptic driver & PWM motor
    pinMode(TEENSY_MOTOR_DRIVER_VCC, OUTPUT);
    pinMode(TEENSY_MOTOR_DRIVER_PH, OUTPUT);
    pinMode(TEENSY_MOTOR_DRIVER_EN, OUTPUT);
    pinMode(TEENSY_LED_PIN, OUTPUT);

    digitalWrite(TEENSY_MOTOR_DRIVER_VCC, HIGH);
    digitalWrite(TEENSY_MOTOR_DRIVER_PH, HIGH);

    // Check I2S
    /*  OLD HEADSET
        TEENSY_BCLK1: 1
        TEENSY_LRCLK1: 1
        TEENSY_BCLK2: 0
        TEENSY_LRCLK2: 1
        TEENSY_MCLK2: 0

        NEW HEADSET
        TEENSY_BCLK1: 1
        TEENSY_LRCLK1: 1
        TEENSY_BCLK2: 0
        TEENSY_LRCLK2: 1
        TEENSY_MCLK2: 0
    */
    // pinMode(TEENSY_MCLK2, INPUT); // 0 on new headset and old headset
    // pinMode(TEENSY_BCLK1, INPUT);
    // pinMode(TEENSY_LRCLK1, INPUT);
    // pinMode(TEENSY_BCLK2, INPUT);
    // pinMode(TEENSY_LRCLK2, INPUT);
    // Serial.print("TEENSY_BCLK1: "); Serial.println(digitalRead(TEENSY_BCLK1));
    // Serial.print("TEENSY_LRCLK1: "); Serial.println(digitalRead(TEENSY_LRCLK1));
    // Serial.print("TEENSY_BCLK2: "); Serial.println(digitalRead(TEENSY_BCLK2));
    // Serial.print("TEENSY_LRCLK2: "); Serial.println(digitalRead(TEENSY_LRCLK2));
    // Serial.print("TEENSY_MCLK2: "); Serial.println(digitalRead(TEENSY_MCLK2));

    // Get spl value from the SD card
    if ((err = read_spl_limits_from_file()) != ERR_SAMMS_OK) {
        Serial.print("Could not read "); Serial.print(sysConfig.splUserConfig.fileName);
        Serial.println(" from the SD card.");
    }

    // Turn on accelerometer and set specifications
    if ((err = setup_accel()) != ERR_SAMMS_OK) {
        Serial.println("Accelerometer begin failed.");
    } else {
        Serial.println("Accelerometer specifications set.");
    }

    return err;
}

void samms_open_file_rw (void) {

//  if (SD.exists("rawaccelx.txt")) {
//     SD.remove("rawaccelx.txt");
//   }
//   sysData.files.faccelx = SD.open("rawaccelx.txt", FILE_WRITE);
//   if (sysData.files.faccelx) {
//     Serial.println("File Open X");
//   }

//   if (SD.exists("rawaccely.txt")) {
//     SD.remove("rawaccely.txt");
//   }
//   sysData.files.faccely = SD.open("rawaccely.txt", FILE_WRITE);
//   if (sysData.files.faccely) {
//     Serial.println("File Open Y");
//   }

//   if (SD.exists("rawaccelz.txt")) {
//     SD.remove("rawaccelz.txt");
//   }
//   sysData.files.faccelz = SD.open("rawaccelz.txt", FILE_WRITE);
//   if (sysData.files.faccelz) {
//     Serial.println("File Open Z");
//   }

//   if (SD.exists("MICRAWNEW_newheadset.RAW")) {
//     SD.remove("MICRAWNEW_newheadset.RAW");
//   }
//   sysData.files.fmicraw = SD.open("MICRAWNEW_newheadset.RAW", FILE_WRITE);
//   if (sysData.files.fmicraw) {
//     Serial.println("File Open MICRAWNEW_newheadset");
//   }

  if (SD.exists("MICNLMSOUT.RAW")) {
    SD.remove("MICNLMSOUT.RAW");
  }
  sysData.files.fnlms = SD.open("MICNLMSOUT.RAW", FILE_WRITE);
  if (sysData.files.fnlms) {
    Serial.println("File Open MICNLMSOUT");
  }

}

bool is_buzz_timer_expired() {
    if (sysData.buzzOnTimeStamp_ms != 0 && millis() - sysData.buzzOnTimeStamp_ms >= 500) {
        return true;
    } else {
        return false;
    }
}

bool samms_toggle_buzz (bool turnOn) {
    if (turnOn) {
        if (!sysStatus.isMotorOn) {
            Serial.println("BUZZING!");
            sysData.buzzOnTimeStamp_ms = millis();
            analogWrite(TEENSY_MOTOR_DRIVER_EN, 0);
            analogWrite(TEENSY_MOTOR_DRIVER_EN, 70);
            sysStatus.isMotorOn = true;
        }
    } else {
        if (sysStatus.isMotorOn) {
            Serial.println("Buzz Off");
            analogWrite(TEENSY_MOTOR_DRIVER_EN, 0);
            sysStatus.isMotorOn = false;
        }
    }
    return sysStatus.isMotorOn;
}