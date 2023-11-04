#include <ArduinoBLE.h>

#define USING_TIMER_TC3         true      // Only TC3 can be used for SAMD51
#define USING_TIMER_TC4         false     // Not to use with Servo library
#define USING_TIMER_TC5         false
#define USING_TIMER_TCC         false
#define USING_TIMER_TCC1        false
#define USING_TIMER_TCC2        false     // Don't use this, can crash on some boards

#include "SAMDTimerInterrupt.h"

/////////////////////////////////////////////////////////////////

#define TIMER_INTERVAL_MS        0.833

///////////////////////////////////////////////

#if (TIMER_INTERRUPT_USING_SAMD21)

  #if USING_TIMER_TC3
    #define SELECTED_TIMER      TIMER_TC3
  #elif USING_TIMER_TC4
    #define SELECTED_TIMER      TIMER_TC4
  #elif USING_TIMER_TC5
    #define SELECTED_TIMER      TIMER_TC5
  #elif USING_TIMER_TCC
    #define SELECTED_TIMER      TIMER_TCC
  #elif USING_TIMER_TCC1
    #define SELECTED_TIMER      TIMER_TCC1
  #elif USING_TIMER_TCC2
    #define SELECTED_TIMER      TIMER_TCC
  #else
    #error You have to select 1 Timer  
  #endif

#else

  #if !(USING_TIMER_TC3)
    #error You must select TC3 for SAMD51
  #endif
  
  #define SELECTED_TIMER      TIMER_TC3

#endif  

// Init selected SAMD timer
SAMDTimer ITimer(SELECTED_TIMER);

BLEService emgService("731AE476-51DB-4D71-BFCC-3FA8CB857417");
BLEByteCharacteristic calibrationCharacteristic("736086C4-6585-41AB-A0C1-5F20187093F8", BLERead | BLEWrite);
BLEFloatCharacteristic emgCharacteristic("2CCED713-B7B6-4783-AAF9-3C138EB8BE08", BLERead | BLENotify);

int calibrationFlag = 0;
float mvc = 0;
float offset = 0;

const int n = 128;
const int samplingFrequency = 1200;

const unsigned long sampleInterval = 833;
const unsigned long windowWidth = 25000;

float a[n];
float f[n];
const int bufferSize = 3;
float buffer[bufferSize];
int bufferIndex = 0;

float rmsBuffer[n];

int i = 0;
int j = 0;
int k = 0;

unsigned long previousMicros = 0;


bool flag = false;

float A[] = { 1,         -2.65247741,  0.66214634,  2.59580099,  0.24226609, -3.06708861, -0.14553671,  1.62620564,  0.32596455, -0.53334254, -0.15443221,  0.07297218, 0.02753774};
float B[] = {0.16593348,  0,         -0.99560088, 0,          2.48900221,  0, -3.31866961,  0,          2.48900221,  0,         -0.99560088,  0,  0.16593348};

enum State {
  CALIBRATION,
  NORMAL_OPERATION
};

State currentState = CALIBRATION;

void TimerHandler()
{
  if(k < n)
  {
    a[k] = ((analogRead(A0) * (3.3/1023.0)) - offset)/mvc;
    if (k == 0){
      f[k] = (- A[1]*f[127] - A[2]*f[126] - A[3]*f[125] - A[4]*f[124] - A[5]*f[123] - A[6]*f[122] - A[7]*f[121] - A[8]*f[120] - A[9]*f[119] - A[10]*f[118] - A[11]*f[117] - A[12]*f[116] + B[0]*a[k] + B[1]*a[127] + B[2]*a[126] + B[3]*a[125] + B[4]*a[124] + B[5]*a[123] + B[6]*a[122] + B[7]*a[121] + B[8]*a[120] + B[9]*a[119] + B[10]*a[118] + B[11]*a[117] + B[12]*a[116])/A[0];
    }
    else if(k == 1){
      f[k] = (- A[1]*f[k-1] - A[2]*f[127] - A[3]*f[126] - A[4]*f[125] - A[5]*f[124] - A[6]*f[123] - A[7]*f[122] - A[8]*f[121] - A[9]*f[120] - A[10]*f[119] - A[11]*f[118] - A[12]*f[117] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[127] + B[3]*a[126] + B[4]*a[125] + B[5]*a[124] + B[6]*a[123] + B[7]*a[122] + B[8]*a[121] + B[9]*a[120] + B[10]*a[119] + B[11]*a[118] + B[12]*a[117])/A[0];
    }
    else if(k == 2){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[127] - A[4]*f[126] - A[5]*f[125] - A[6]*f[124] - A[7]*f[123] - A[8]*f[122] - A[9]*f[121] - A[10]*f[120] - A[11]*f[119] - A[12]*f[118] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[127] + B[4]*a[126] + B[5]*a[125] + B[6]*a[124] + B[7]*a[123] + B[8]*a[122] + B[9]*a[121] + B[10]*a[120] + B[11]*a[119] + B[12]*a[118])/A[0];
    }
    else if(k == 3){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[127] - A[5]*f[126] - A[6]*f[125] - A[7]*f[124] - A[8]*f[123] - A[9]*f[122] - A[10]*f[121] - A[11]*f[120] - A[12]*f[119] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[127] + B[5]*a[126] + B[6]*a[125] + B[7]*a[124] + B[8]*a[123] + B[9]*a[122] + B[10]*a[121] + B[11]*a[120] + B[12]*a[119])/A[0];
    }
    else if(k == 4){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[127] - A[6]*f[126] - A[7]*f[125] - A[8]*f[124] - A[9]*f[123] - A[10]*f[122] - A[11]*f[121] - A[12]*f[120] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[127] + B[6]*a[126] + B[7]*a[125] + B[8]*a[124] + B[9]*a[123] + B[10]*a[122] + B[11]*a[121] + B[12]*a[120])/A[0];
    }
    else if(k == 5){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[127] - A[7]*f[126] - A[8]*f[125] - A[9]*f[124] - A[10]*f[123] - A[11]*f[122] - A[12]*f[121] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[127] + B[7]*a[126] + B[8]*a[125] + B[9]*a[124] + B[10]*a[123] + B[11]*a[122] + B[12]*a[121])/A[0];
    }
    else if(k == 6){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[127] - A[8]*f[126] - A[9]*f[125] - A[10]*f[124] - A[11]*f[123] - A[12]*f[122] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[127] + B[8]*a[126] + B[9]*a[125] + B[10]*a[124] + B[11]*a[123] + B[12]*a[122])/A[0];
    }
    else if(k == 7){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[127] - A[9]*f[126] - A[10]*f[125] - A[11]*f[124] - A[12]*f[123] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[127] + B[9]*a[126] + B[10]*a[125] + B[11]*a[124] + B[12]*a[123])/A[0];
    }
    else if(k == 8){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[127] - A[10]*f[126] - A[11]*f[125] - A[12]*f[124] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[127] + B[10]*a[126]+ B[11]*a[125] + B[12]*a[124])/A[0];
    }
    else if(k == 9){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[k-9] - A[10]*f[127] - A[11]*f[126] - A[12]*f[125] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[k-9] + B[10]*a[127] + B[11]*a[126] + B[12]*a[125])/A[0];
    }
    else if(k == 10){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[k-9] - A[10]*f[k-10] - A[11]*f[127] - A[12]*f[126] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[k-9] + B[10]*a[k-10] + B[11]*a[127] + B[12]*a[126])/A[0];
    }
    else if(k == 11){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[k-9] - A[10]*f[k-10] - A[11]*f[k-11] - A[12]*f[127] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[k-9] + B[10]*a[k-10] + B[11]*a[k-11] + B[12]*a[127])/A[0];
    }
    else{
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[k-9] - A[10]*f[k-10] - A[11]*f[k-11] - A[12]*f[k-12] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[k-9] + B[10]*a[k-10] + B[11]*a[k-11] + B[12]*a[k-12])/A[0];
    }
    float absValue = abs(f[k]);
    rmsBuffer[k] = absValue * absValue;
    k++;
   
    
    //When 100 samples are acquired and filtered, the flag is activated
    if(k == n)
    {
      k = 0;
      flag = true;
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("Bluetooth Error!");
    while (1);
  }

  BLE.setLocalName("Left Module");
  BLE.setAdvertisedService(emgService);
  emgService.addCharacteristic(calibrationCharacteristic);
  emgService.addCharacteristic(emgCharacteristic);
  BLE.addService(emgService);
  BLE.advertise();
  Serial.println("BLE server started");

  // Set initial state to calibration
  currentState = CALIBRATION;

}

void loop() {
  BLEDevice central = BLE.central();

  switch (currentState) {
    case CALIBRATION:
      calibration(central, offset);
      break;

    case NORMAL_OPERATION:
      // Your main code for normal operation goes here
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
      break;
  }
}

void calibration(BLEDevice central, float o) {
  if (central && central.connected()) {
    if (calibrationCharacteristic.written()) {
      uint8_t receivedString = calibrationCharacteristic.value();
      if (receivedString == 1) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Calibration started!!!");
        for(int i = 0; i < 3600; i++)
        {
          
          float read_value = (analogRead(A0)*(3.3/1023.0)) - o;

          if(read_value > mvc)
          {
            mvc = read_value;
          }
          
          delayMicroseconds(593);
        }
        
        digitalWrite(LED_BUILTIN, LOW);
        Serial1.print("MVC Value: ");
        Serial1.println(mvc);
        Serial.println("Calibration Sucessfull!");
        calibrationFlag = 1;

        ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, TimerHandler);
        currentState = NORMAL_OPERATION; // Transition to normal operation state
      }
    }
  }
}


 
