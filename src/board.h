#ifndef board_h_
#define board_h_

#include <Arduino.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

// ERRORS
#define ERR_SAMMS_OK                 0
#define ERR_FILE_SYS_SD_FAIL        -1
#define ERR_FILE_SYS_RW_FAIL        -2

// Spl file and limits
typedef struct {
    char fileName[32];
    float splLowerdBA;
    float splUpperdBA;
} spl_user_config_t;

// System configuration values
typedef struct {
    spl_user_config_t splUserConfig;
    Adafruit_ADXL343  accel;
} samms_sys_config_t;

// Decibel statistics from the mic
typedef struct {
  float sum;
  float avg;
  int count;
} decibel_stats_t;

// System data
typedef struct {
    decibel_stats_t dBStats;
} samms_sys_data_t;

// System status
typedef struct {
    bool isMotorOn;
} samms_sys_status_t;

extern samms_sys_config_t   sysConfig;
extern samms_sys_data_t     sysData;
extern samms_sys_status_t   sysStatus;

#endif