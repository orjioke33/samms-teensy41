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
    },
    .micEnergyData = {
        .magnitude = 0,
        .diffMagSq = 0,
        .noiseConstant = 0.5,
    },
    .uptimeSeconds = 0,
};

// sysStatus
samms_sys_status_t sysStatus = {
    .isMotorOn = false,
    .isSpeechDetected = false,
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

 if (SD.exists("rawaccelx.txt")) {
    SD.remove("rawaccelx.txt");
  }
  sysData.files.faccelx = SD.open("rawaccelx.txt", FILE_WRITE);
  if (sysData.files.faccelx) {
    Serial.println("File Open X");
  }

  if (SD.exists("rawaccely.txt")) {
    SD.remove("rawaccely.txt");
  }
  sysData.files.faccely = SD.open("rawaccely.txt", FILE_WRITE);
  if (sysData.files.faccely) {
    Serial.println("File Open Y");
  }

  if (SD.exists("rawaccelz.txt")) {
    SD.remove("rawaccelz.txt");
  }
  sysData.files.faccelz = SD.open("rawaccelz.txt", FILE_WRITE);
  if (sysData.files.faccelz) {
    Serial.println("File Open Z");
  }

  if (SD.exists("MICLEFT.RAW")) {
    SD.remove("MICLEFT.RAW");
  }
  sysData.files.fmicleftraw = SD.open("MICLEFT.RAW", FILE_WRITE);
  if (sysData.files.fmicleftraw) {
    Serial.println("File Open MICLEFT.RAW");
  }

  if (SD.exists("MICRIGHT.RAW")) {
    SD.remove("MICRIGHT.RAW");
  }
  sysData.files.fmicrightraw = SD.open("MICRIGHT.RAW", FILE_WRITE);
  if (sysData.files.fmicrightraw) {
    Serial.println("File Open MICRIGHT.RAW");
  }

  if (SD.exists("MICSUM.RAW")) {
    SD.remove("MICSUM.RAW");
  }
  sysData.files.fmicraw = SD.open("MICSUM.RAW", FILE_WRITE);
  if (sysData.files.fmicraw) {
    Serial.println("File Open MICSUM.RAW");
  }

  if (SD.exists("NLMS_RECORD.RAW")) {
    SD.remove("NLMS_RECORD.RAW");
  }
  sysData.files.fnlms = SD.open("NLMS_RECORD.RAW", FILE_WRITE);
  if (sysData.files.fnlms) {
    Serial.println("File Open NLMS_RECORD.RAW");
  }

  if (SD.exists("FFT_RECORD.RAW")) {
    SD.remove("FFT_RECORD.RAW");
  }
  sysData.files.ffft = SD.open("FFT_RECORD.RAW", FILE_WRITE);
  if (sysData.files.ffft) {
    Serial.println("File Open FFT_RECORD.RAW");
  }
}

bool samms_toggle_buzz (bool turnOn) {
    if (turnOn) {
        if (!sysStatus.isMotorOn) {
            Serial.println("BUZZING!");
            analogWrite(TEENSY_MOTOR_DRIVER_EN, 0);
            analogWrite(TEENSY_MOTOR_DRIVER_EN, 70);
            sysStatus.isMotorOn = true;
        }
    } else {
        if (sysStatus.isMotorOn) {
            analogWrite(TEENSY_MOTOR_DRIVER_EN, 0);
            sysStatus.isMotorOn = false;
        }
    }
    return sysStatus.isMotorOn;
}