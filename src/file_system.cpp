#include "board.h"
#include "file_system.h"

// Reads the upper and lower dBA limits from
// the "spl.txt" file
int8_t read_spl_limits_from_file (void) {

  int8_t err = ERR_SAMMS_OK;
  File frec;

  // Check if there is an SD card
  if (!(SD.begin(BUILTIN_SDCARD))) {
    Serial.println("Unable to access SD card.");
    err = ERR_FILE_SYS_SD_FAIL;
    goto quit;
  }

  frec = SD.open(sysConfig.splUserConfig.fileName, FILE_READ);
  if (!frec) {
    Serial.println("Error opening spl txt file.");
    err = ERR_FILE_SYS_RW_FAIL;
  } else {
    sysConfig.splUserConfig.splLowerdBA = frec.parseFloat();
    sysConfig.splUserConfig.splUpperdBA = frec.parseFloat();
    Serial.println("Lower and upper threshold respectively (dB)");
    Serial.println(sysConfig.splUserConfig.splLowerdBA, 3);
    Serial.println(sysConfig.splUserConfig.splUpperdBA, 3);
  }

quit:
  return err;
}