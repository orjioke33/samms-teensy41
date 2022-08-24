// https://forum.pjrc.com/threads/46150-Recording-Stereo-Audio-to-SD-Card?p=158682&viewfull=1#post158682
// https://forum.pjrc.com/threads/42562-MEMS-i2c-microphone-SPH0645LM4H-with-teensy-audio-library?p=160880&viewfull=1#post160880
// 16 bit 44.1khz

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include "board.h"
#include "file_system.h"
#include "nlms.h"
#include "arm_math.h"
#include "sqrt_integer.h"

// i2s1 and i2s2 are already wired for Teensy boards
AudioInputI2S            i2sL;           //xy=168,145
AudioInputI2S2           i2sR;           //xy=168,145
AudioRecordQueue         queue1;         //xy=360,62
AudioRecordQueue         queue2;         //xy=389,145
AudioConnection          patchCord1(i2sL, 0, queue1, 0); // Left Channel
AudioConnection          patchCord2(i2sR, 0, queue2, 0); // Right Channel

const int LED_PIN = 13;
//config ports for motor driver
const int MOTOR_DRIVER_VCC = 37;
const int MOTOR_DRIVER_PH = 38;
//PWM port
const int MOTOR_DRIVER_EN = 29;


#define BUFFER_SIZE_MIC         512

// nlms
short micLeftBuffer[BUFFER_SIZE_MIC];
short micRightBuffer[BUFFER_SIZE_MIC];
double micSum[2*BUFFER_SIZE_MIC]={0};
double micDiff[2*BUFFER_SIZE_MIC]={0};
double nlmsOut[BUFFER_SIZE_MIC];
double nlms_weights[128] = {0};
float32_t aWeight[512] = {0.000491601, 0.007439144, 0.026456683, 0.057237193, 0.097648235, 0.145530499, 0.19895028, 0.256192702, 0.315753433, 0.376341431, 0.436880336, 0.496502258, 0.554533348, 0.610473237, 0.663971116, 0.714800959, 0.762837753, 0.808035908, 0.85041046, 0.890021233, 0.926959868, 0.961339445, 0.993286373, 1.022934171, 1.050418819, 1.075875354, 1.099435459, 1.12122582, 1.141367062, 1.159973138, 1.177151032, 1.193000704, 1.207615207, 1.221080906, 1.233477786, 1.244879798, 1.255355228, 1.264967081, 1.273773452, 1.281827897, 1.289179783, 1.29587462, 1.301954369, 1.307457742, 1.312420463, 1.316875521, 1.320853401, 1.324382291, 1.327488281, 1.330195534, 1.332526449, 1.334501812, 1.336140927, 1.337461742, 1.338480956, 1.339214128, 1.339675764, 1.339879407, 1.33983771, 1.33956251, 1.339064892, 1.338355246, 1.337443324, 1.336338287, 1.33504875, 1.333582824, 1.331948154, 1.330151954, 1.328201037, 1.326101843, 1.323860472, 1.3214827, 1.318974007, 1.316339599, 1.313584419, 1.310713175, 1.307730349, 1.304640214, 1.301446848, 1.298154147, 1.294765835, 1.291285478, 1.28771649, 1.284062146, 1.280325589, 1.276509835, 1.272617788, 1.268652238, 1.264615874, 1.260511287, 1.256340974, 1.252107348, 1.247812737, 1.243459393, 1.239049492, 1.234585143, 1.230068386, 1.2255012, 1.220885501, 1.216223152, 1.211515958, 1.206765674, 1.201974005, 1.19714261, 1.1922731, 1.187367045, 1.182425973, 1.177451372, 1.172444693, 1.167407348, 1.162340715, 1.157246139, 1.152124932, 1.146978373, 1.141807712, 1.136614169, 1.131398936, 1.126163178, 1.120908033, 1.115634612, 1.110344003, 1.105037268, 1.099715447, 1.094379556, 1.08903059, 1.083669521, 1.078297301, 1.072914859, 1.067523108, 1.062122937, 1.05671522, 1.05130081, 1.045880542, 1.040455233, 1.035025683, 1.029592675, 1.024156975, 1.018719333, 1.013280482, 1.00784114, 1.00240201, 0.996963778, 0.991527118, 0.986092686, 0.980661126, 0.975233068, 0.969809127, 0.964389904, 0.958975988, 0.953567954, 0.948166364, 0.942771769, 0.937384705, 0.932005696, 0.926635255, 0.921273883, 0.915922069, 0.91058029, 0.905249011, 0.899928687, 0.894619763, 0.88932267, 0.88403783, 0.878765656, 0.873506548, 0.868260896, 0.863029082, 0.857811477, 0.852608441, 0.847420326, 0.842247473, 0.837090216, 0.831948877, 0.826823771, 0.821715204, 0.816623471, 0.811548862, 0.806491654, 0.80145212, 0.796430522, 0.791427115, 0.786442146, 0.781475853, 0.776528467, 0.771600212, 0.766691304, 0.761801951, 0.756932355, 0.752082709, 0.747253202, 0.742444013, 0.737655316, 0.732887277, 0.728140056, 0.723413808, 0.71870868, 0.714024813, 0.709362343, 0.704721397, 0.700102099, 0.695504567, 0.690928912, 0.686375241, 0.681843653, 0.677334244, 0.672847104, 0.668382317, 0.663939963, 0.659520116, 0.655122846, 0.650748219, 0.646396293, 0.642067124, 0.637760764, 0.633477258, 0.629216649, 0.624978975, 0.620764269, 0.616572561, 0.612403877, 0.608258238, 0.604135661, 0.600036161, 0.595959749, 0.59190643, 0.587876209, 0.583869084, 0.579885053, 0.575924108, 0.571986239, 0.568071433, 0.564179674, 0.560310942, 0.556465216, 0.552642469, 0.548842674, 0.5450658, 0.541311815, 0.537580682, 0.533872363, 0.530186816, 0.526524, 0.522883867, 0.519266371, 0.51567146, 0.512099084, 0.508549187, 0.505021714, 0.501516605, 0.498033801, 0.49457324, 0.491134857, 0.487718587, 0.484324362, 0.480952113, 0.47760177, 0.47427326, 0.47096651, 0.467681443, 0.464417985, 0.461176056, 0.457955577, 0.454756467, 0.451578645, 0.448422028, 0.445286531, 0.442172069, 0.439078556, 0.436005904, 0.432954025, 0.429922829, 0.426912226, 0.423922125, 0.420952434, 0.41800306, 0.41507391, 0.412164889, 0.409275902, 0.406406853, 0.403557647, 0.400728185, 0.397918372, 0.395128108, 0.392357295, 0.389605834, 0.386873625, 0.384160569, 0.381466565, 0.378791512, 0.37613531, 0.373497856, 0.37087905, 0.368278789, 0.36569697, 0.363133492, 0.360588252, 0.358061147, 0.355552074, 0.353060929, 0.35058761, 0.348132012, 0.345694033, 0.343273569, 0.340870517, 0.338484772, 0.336116231, 0.333764791, 0.331430348, 0.329112798, 0.326812038, 0.324527964, 0.322260474, 0.320009464, 0.31777483, 0.315556471, 0.313354282, 0.311168162, 0.308998008, 0.306843717, 0.304705188, 0.302582319, 0.300475007, 0.298383152, 0.296306653, 0.294245408, 0.292199316, 0.290168278, 0.288152193, 0.286150961, 0.284164484, 0.28219266, 0.280235393, 0.278292582, 0.276364131, 0.27444994, 0.272549913, 0.270663953, 0.268791961, 0.266933843, 0.265089501, 0.263258841, 0.261441767, 0.259638183, 0.257847996, 0.256071111, 0.254307435, 0.252556875, 0.250819337, 0.249094729, 0.247382959, 0.245683935, 0.243997567, 0.242323763, 0.240662434, 0.239013489, 0.23737684, 0.235752397, 0.234140071, 0.232539776, 0.230951422, 0.229374924, 0.227810195, 0.226257148, 0.224715697, 0.223185759, 0.221667247, 0.220160077, 0.218664167, 0.217179432, 0.215705789, 0.214243158, 0.212791454, 0.211350598, 0.209920507, 0.208501103, 0.207092304, 0.205694032, 0.204306207, 0.202928752, 0.201561587, 0.200204635, 0.19885782, 0.197521065, 0.196194293, 0.19487743, 0.193570399, 0.192273127, 0.19098554, 0.189707563, 0.188439124, 0.187180149, 0.185930567, 0.184690306, 0.183459294, 0.182237461, 0.181024737, 0.179821051, 0.178626335, 0.177440519, 0.176263535, 0.175095315, 0.173935792, 0.172784898, 0.171642567, 0.170508733, 0.16938333, 0.168266293, 0.167157558, 0.166057059, 0.164964734, 0.163880519, 0.162804351, 0.161736168, 0.160675908, 0.159623508, 0.158578909, 0.157542048, 0.156512867, 0.155491306, 0.154477304, 0.153470803, 0.152471745, 0.151480071, 0.150495724, 0.149518647, 0.148548783, 0.147586076, 0.146630469, 0.145681907, 0.144740336, 0.143805699, 0.142877944, 0.141957016, 0.141042862, 0.140135428, 0.139234663, 0.138340513, 0.137452928, 0.136571854, 0.135697243, 0.134829041, 0.133967201, 0.133111671, 0.132262402, 0.131419345, 0.130582451, 0.129751671, 0.128926959, 0.128108265, 0.127295544, 0.126488747, 0.125687829, 0.124892743, 0.124103444, 0.123319885, 0.122542022, 0.121769811, 0.121003206, 0.120242164, 0.119486641, 0.118736594, 0.117991979, 0.117252754, 0.116518877, 0.115790306, 0.115066999, 0.114348914, 0.113636011, 0.112928249, 0.112225587, 0.111527986, 0.110835406, 0.110147807, 0.109465151, 0.108787398, 0.108114511, 0.10744645, 0.106783179, 0.106124659, 0.105470853, 0.104821725, 0.104177238, 0.103537355, 0.10290204, 0.102271258, 0.101644973, 0.10102315, 0.100405754, 0.09979275, 0.099184104, 0.098579782, 0.09797975, 0.097383975, 0.096792423, 0.096205061, 0.095621857, 0.095042778, 0.094467792, 0.093896867, 0.093329972, 0.092767075, 0.092208144, 0.091653149, 0.09110206, 0.090554845, 0.090011475, 0.08947192, 0.088936151, 0.088404136, 0.087875849, 0.087351259, 0.086830338};
float32_t bp_weight[512];
// fft
float32_t buffer[1024] = {0};
float32_t cmplx_mag[512] = {0};
float32_t output[512] __attribute__ ((aligned (4)));
//arm_cfft_radix2_instance_f32 fft_inst;
arm_cfft_instance_f32 fft_inst;
extern "C" {
extern const int16_t AudioWindowHanning1024[];
extern const int16_t AudioWindowBartlett1024[];
extern const int16_t AudioWindowBlackman1024[];
extern const int16_t AudioWindowFlattop1024[];
extern const int16_t AudioWindowBlackmanHarris1024[];
extern const int16_t AudioWindowNuttall1024[];
extern const int16_t AudioWindowBlackmanNuttall1024[];
extern const int16_t AudioWindowWelch1024[];
extern const int16_t AudioWindowHamming1024[];
extern const int16_t AudioWindowCosine1024[];
extern const int16_t AudioWindowTukey1024[];
}

float32_t v[512] = {0};
float32_t magnitude = 0;
float32_t dB_holder;

// Use these with the Teensy 3.5 & 3.6 SD card
#define SDCARD_CS_PIN    BUILTIN_SDCARD // 254?
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording, 2=playing

// The file where data is recorded
File frec;
struct dBStats {
  float sum;
  float avg;
  int count;
} dBStat;
bool motorOn = false;

// Spl file values
float dBLower = 0;
float dBUpper = 0;

void buzzOn () {
  if (!motorOn) {
    Serial.println("BUZZING!");
    analogWrite(MOTOR_DRIVER_EN, 0);
    analogWrite(MOTOR_DRIVER_EN, 70);
    motorOn = true;
  }
}

void buzzOff () {
  if (motorOn) {
    Serial.println("OFFF!");
    analogWrite(MOTOR_DRIVER_EN, 0);
    motorOn = false;
  }
}

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

// write all 512 bytes to the SD card   
void continueRecording() {
  if (queue1.available() >= 4 && queue2.available() >= 4) {
    double micNoise[BUFFER_SIZE_MIC];

    // If >= 1024 bytes of mic data is available, save in a buffer
    memcpy(micLeftBuffer, queue1.readBuffer(), BUFFER_SIZE_MIC);
    memcpy(micRightBuffer, queue2.readBuffer(), BUFFER_SIZE_MIC);
    queue1.freeBuffer();
    queue2.freeBuffer();

    // copy second half to first half
    for (int i = 0; i < BUFFER_SIZE_MIC; i++) {
      micDiff[i] = micDiff[i + BUFFER_SIZE_MIC];
      micSum[i] = micSum[i + BUFFER_SIZE_MIC];
    }

  //store new data in second helf
    for (int i = 0; i < BUFFER_SIZE_MIC; i++) {
      micDiff[i+BUFFER_SIZE_MIC] = micLeftBuffer[i] - micRightBuffer[i];
      //Serial.println(micLeftBuffer[i],10);
      micSum[i+BUFFER_SIZE_MIC] = micLeftBuffer[i] + micRightBuffer[i];
    }


    // Filter background noise
    do_nlms(micDiff, micSum, micNoise, nlmsOut, nlms_weights, 0.1, 128, BUFFER_SIZE_MIC);

    copy_to_fft_buffer(buffer, nlmsOut);
    //arm_cfft_radix2_f32(&fft_inst, buffer);
    arm_cfft_f32(&fft_inst, buffer, 0, 1);//no bit reverse

    //scale and calculate complex magnitude squared
    //for (int i=0; i < 512; i++) {
    //    float32_t tmp = *((float32_t *)buffer + i); // real & imag
    //    float32_t magsq = multiply_16tx16t_add_16bx16b(tmp, tmp);
    //    output[i] = sqrt_uint32_approx(magsq) * (1.0f / 16384.0f);
    //}

    arm_cmplx_mag_squared_f32(buffer,output,512);

    //BP and aweight filtering
    magnitude = 0;
    dB_holder = 0;
      
    for (int i=0; i<512; i++) {
      v[i] = output[i] * bp_weight[i] * 1.0f/16384.0f; //* aWeight[i] * bp_weight[i] * 1.0f/16384.0f; //* 1.0f/262144.0f;//* 1/(512^2)
      magnitude = magnitude + v[i]; // + sq(v[i]);
    }

    //get spl and buzz?
  
    dB_holder = log10f(magnitude) * 10  + 125.05;
    dBStat.sum += dB_holder;
    dBStat.count++;
    dBStat.avg = dBStat.sum / dBStat.count;

    Serial.println(nlmsOut[0], 6);
    
    // Check for buzz every 96 dB samples
    if (dBStat.count >= 96) {
      /*
      if (dBStat.avg > dBLower && dBStat.avg < dBUpper && !motorOn) {
          Serial.println(dBStat.avg,2); // f[23] = 1kHz, f[82] = 3.5kHz, f[252] = 12kHz
          buzzOn(); delay(250); buzzOff();
      }*/
      dBStat.sum = 0;
      dBStat.count = 0;
      dBStat.avg = 0;
      }

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

  //arm_cfft_radix2_init_f32(&fft_inst,1024,0,1); 

  //bins [7 42] should have weights 1 for BP filtering
  for(int i=0; i<512; i++) {
    if(i > 42){
      bp_weight[i] = 0.0;
    } else if(i > 6) {
      bp_weight[i] = 1.0;
    } else {
      bp_weight[i] = 0.0;
    }
  }

  // Do not continue without an SD card
  while (read_spl_limits_from_file() != ERR_SAMMS_OK) {
  }

  // Setup haptic driver & PWM motor
  pinMode(MOTOR_DRIVER_VCC, OUTPUT);
  pinMode(MOTOR_DRIVER_PH, OUTPUT);
  pinMode(MOTOR_DRIVER_EN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(MOTOR_DRIVER_VCC, HIGH);
  digitalWrite(MOTOR_DRIVER_PH, HIGH);

  dBStat.avg = 0, dBStat.sum = 0, dBStat.sum = 0;

  startRecording();
}


void loop() {
  if (millis() > 30000 && mode == 1) {
    stopRecording();
  }
  else {
    if (mode == 1) continueRecording();
  }
}