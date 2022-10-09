#ifndef board_h_
#define board_h_

#include <Arduino.h>
#include <Audio.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

// PINOUTS
#define TEENSY_LED_PIN              13
#define TEENSY_MOTOR_DRIVER_VCC     37
#define TEENSY_MOTOR_DRIVER_PH      38
#define TEENSY_MOTOR_DRIVER_EN      29 
#define TEENSY_MCLK2                33
#define TEENSY_BCLK1                21
#define TEENSY_LRCLK1               20
#define TEENSY_BCLK2                4
#define TEENSY_LRCLK2               3        

// ERRORS
#define ERR_SAMMS_OK                 0
#define ERR_FILE_SYS_SD_FAIL        -1
#define ERR_FILE_SYS_RW_FAIL        -2
#define ERR_ACCEL_BEGIN_FAIL        -1

#define DEFAULT_ACCEL_BUFFER_SIZE   512
#define BUFFER_SIZE_MIC             128
#define DELAYOFFSET                 64

// TESTS
#define RUN_AVERAGE_DB_TEST         false
#define DEFAULT_CLINICAL_TRIAL_LENGTH_SECONDS   1800 // 30 minutes

// Spl file and limits
typedef struct {
    char    fileName[32];
    float   splLowerdBA;
    float   splUpperdBA;
} spl_user_config_t;

// PJRC audio library objects
typedef struct {
    AudioInputI2S            i2sL;           // left mic
    AudioInputI2S2           i2sR;           // right mic
    AudioRecordQueue         queue1;         // left mic raw buf
    AudioRecordQueue         queue2;         // right mic raw buf
} mic_config_t;

// Microphone buffers
typedef struct {
    double yL[BUFFER_SIZE_MIC];
    double yR[BUFFER_SIZE_MIC];
    double micSum[2*BUFFER_SIZE_MIC];
    double micDiff[2*BUFFER_SIZE_MIC];
    double nlms_weights[100];
    short micLeftBuffer[BUFFER_SIZE_MIC+128];//for offset
    short micRightBuffer[BUFFER_SIZE_MIC+128];
    double nlmsOut[BUFFER_SIZE_MIC];
    float32_t bp_weight[512];
} mic_filter_config_t;

// Arm fft objects and buffers
typedef struct {
    float32_t               buffer[1024];
    float32_t               cmplx_mag[512];
    float32_t               output[512] __attribute__ ((aligned (4)));
    arm_cfft_instance_f32   fft_inst;
} fft_config_t;

// System configuration values
typedef struct {
    spl_user_config_t   splUserConfig;
    Adafruit_ADXL343    accel;
    mic_filter_config_t micFilter;
    fft_config_t        fftConfig;
    mic_config_t        mic;
} samms_sys_config_t;

// Decibel statistics from the mic
typedef struct {
  float curr;
  float sum;
  float avg;
  int   count;
  float currSpeaker;
  float sumSpeaker;
  float avgSpeaker;
  int   countSpeaker;
} decibel_stats_t;

typedef struct {
    float32_t magnitude;
    float32_t diffMagSq;
    float32_t noiseConstant;
} mic_energy_calculations_t;

// Accelerometer data
typedef struct {
    int16_t         xRaw[DEFAULT_ACCEL_BUFFER_SIZE];
    int16_t         yRaw[DEFAULT_ACCEL_BUFFER_SIZE];
    int16_t         zRaw[DEFAULT_ACCEL_BUFFER_SIZE];
    int16_t         zCopy[DEFAULT_ACCEL_BUFFER_SIZE];
    sensors_event_t event;
} accel_data_t;

typedef struct {
  File faccelx;
  File faccely;
  File faccelz;
  File fmicraw;
  File fnlms;
  File ffft;
  File fspeakavg;
} file_handler_data_t;

// System data
typedef struct {
    decibel_stats_t             dBStats;
    mic_energy_calculations_t   micEnergyData;
    float32_t                   uptimeSeconds;
    uint32_t                    buzzOnTimeStamp_ms;
    char                        dBAvgBuffer[64];
    accel_data_t                accelData;
    file_handler_data_t         files;
} samms_sys_data_t;

// System status
typedef struct {
    bool isMotorOn;
    bool isSpeechDetected;
    bool runTest;
} samms_sys_status_t;

extern samms_sys_config_t   sysConfig;
extern samms_sys_data_t     sysData;
extern samms_sys_status_t   sysStatus;

int8_t samms_setup(void);
bool samms_toggle_buzz(bool turnOn);
void samms_open_file_rw(void);
bool is_buzz_timer_expired(void);

#endif