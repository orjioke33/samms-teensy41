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
};

// sysData
samms_sys_data_t sysData = {
    .dBStats = {
        .sum = 0,
        .avg = 0,
        .count = 0,
    },
};

// sysStatus
samms_sys_status_t sysStatus = {
    .isMotorOn = false,
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