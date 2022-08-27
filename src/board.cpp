#include "board.h"

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