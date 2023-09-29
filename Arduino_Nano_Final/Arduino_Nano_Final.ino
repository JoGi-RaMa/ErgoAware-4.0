#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <arduinoFFT.h>
#include <Wire.h>

#define potentiometer_address 0x2C

// Select only one to be true for SAMD21. Must must be placed at the beginning before #include "SAMDTimerInterrupt.h"
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

const int n = 100;
const int samplingFrequency = 1200;

const unsigned long sampleInterval = 833;
const unsigned long windowWidth = 10000;
float mvc = 0;
float offset = 0;

float calc_offset()
{
  for(int i = 0; i < 6000; i++)
  {
    
    offset += analogRead(A0)*(3.3/4095.0);
    
    delayMicroseconds(593);
  }

  offset = offset/6000;
  Serial1.print("Offset value:");
  Serial1.println(offset);
  
}

float calibration_sequence(float o)
{
  Serial.println("Calibration started!!!");
  for(int i = 0; i < 3600; i++)
  {
    
    float read_value = (analogRead(A0)*(3.3/4095.0)) - o;

    if(read_value > mvc)
    {
      mvc = read_value;
    }
    
    delayMicroseconds(593);
  }
  
  Serial.println("Calibration Sucessfull!");
  
}

void potenciometer_resistance()
{
  byte instruction_byte = 0b00000000; 
  byte data_bits = 0b11111111; //For a resistance of 1M
  
  Wire.beginTransmission(potentiometer_address);
  Wire.write(instruction_byte);
  Wire.write(data_bits);
  Wire.endTransmission();
}

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

////////////////////////////////////////////////

//Timer Interruption Subroutine
void TimerHandler()
{
  if(k < n)
  {
    
    a[k] = ((analogRead(A0) * (3.3/4095.0)) - offset)/mvc;
    if (k == 0){
      f[k] = (- A[1]*f[99] - A[2]*f[98] - A[3]*f[97] - A[4]*f[96] - A[5]*f[95] - A[6]*f[94] - A[7]*f[93] - A[8]*f[92] - A[9]*f[91] - A[10]*f[90] - A[11]*f[89] - A[12]*f[88] + B[0]*a[k] + B[1]*a[99] + B[2]*a[98] + B[3]*a[97] + B[4]*a[96] + B[5]*a[95] + B[6]*a[94] + B[7]*a[93] + B[8]*a[92] + B[9]*a[91] + B[10]*a[90] + B[11]*a[89] + B[12]*a[88])/A[0];
    }
    else if(k == 1){
      f[k] = (- A[1]*f[k-1] - A[2]*f[99] - A[3]*f[98] - A[4]*f[97] - A[5]*f[96] - A[6]*f[95] - A[7]*f[94] - A[8]*f[93] - A[9]*f[92] - A[10]*f[91] - A[11]*f[90] - A[12]*f[89] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[99] + B[3]*a[98] + B[4]*a[97] + B[5]*a[96] + B[6]*a[95] + B[7]*a[94] + B[8]*a[93] + B[9]*a[92] + B[10]*a[91] + B[11]*a[90] + B[12]*a[89])/A[0];
    }
    else if(k == 2){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[99] - A[4]*f[98] - A[5]*f[97] - A[6]*f[96] - A[7]*f[95] - A[8]*f[94] - A[9]*f[93] - A[10]*f[92] - A[11]*f[91] - A[12]*f[90] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[99] + B[4]*a[98] + B[5]*a[97] + B[6]*a[96] + B[7]*a[95] + B[8]*a[94] + B[9]*a[93] + B[10]*a[92] + B[11]*a[91] + B[12]*a[90])/A[0];
    }
    else if(k == 3){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[99] - A[5]*f[98] - A[6]*f[97] - A[7]*f[96] - A[8]*f[95] - A[9]*f[94] - A[10]*f[93] - A[11]*f[92] - A[12]*f[91] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[99] + B[5]*a[98] + B[6]*a[97] + B[7]*a[96] + B[8]*a[95] + B[9]*a[94] + B[10]*a[93] + B[11]*a[92] + B[12]*a[91])/A[0];
    }
    else if(k == 4){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[99] - A[6]*f[98] - A[7]*f[97] - A[8]*f[96] - A[9]*f[95] - A[10]*f[94] - A[11]*f[93] - A[12]*f[92] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[99] + B[6]*a[98] + B[7]*a[97] + B[8]*a[96] + B[9]*a[95] + B[10]*a[94] + B[11]*a[93] + B[12]*a[92])/A[0];
    }
    else if(k == 5){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[99] - A[7]*f[98] - A[8]*f[97] - A[9]*f[96] - A[10]*f[95] - A[11]*f[94] - A[12]*f[93] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[99] + B[7]*a[98] + B[8]*a[97] + B[9]*a[96] + B[10]*a[95] + B[11]*a[94] + B[12]*a[93])/A[0];
    }
    else if(k == 6){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[99] - A[8]*f[98] - A[9]*f[97] - A[10]*f[96] - A[11]*f[95] - A[12]*f[94] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[99] + B[8]*a[98] + B[9]*a[97] + B[10]*a[96] + B[11]*a[95] + B[12]*a[94])/A[0];
    }
    else if(k == 7){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[99] - A[9]*f[98] - A[10]*f[97] - A[11]*f[96] - A[12]*f[95] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[99] + B[9]*a[98] + B[10]*a[97] + B[11]*a[96] + B[12]*a[95])/A[0];
    }
    else if(k == 8){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[99] - A[10]*f[98] - A[11]*f[97] - A[12]*f[96] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[99] + B[10]*a[98]+ B[11]*a[97] + B[12]*a[96])/A[0];
    }
    else if(k == 9){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[k-9] - A[10]*f[99] - A[11]*f[98] - A[12]*f[97] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[k-9] + B[10]*a[99] + B[11]*a[98] + B[12]*a[97])/A[0];
    }
    else if(k == 10){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[k-9] - A[10]*f[k-10] - A[11]*f[99] - A[12]*f[98] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[k-9] + B[10]*a[k-10] + B[11]*a[99] + B[12]*a[98])/A[0];
    }
    else if(k == 11){
      f[k] = (- A[1]*f[k-1] - A[2]*f[k-2] - A[3]*f[k-3] - A[4]*f[k-4] - A[5]*f[k-5] - A[6]*f[k-6] - A[7]*f[k-7] - A[8]*f[k-8] - A[9]*f[k-9] - A[10]*f[k-10] - A[11]*f[k-11] - A[12]*f[99] + B[0]*a[k] + B[1]*a[k-1] + B[2]*a[k-2] + B[3]*a[k-3] + B[4]*a[k-4] + B[5]*a[k-5] + B[6]*a[k-6] + B[7]*a[k-7] + B[8]*a[k-8] + B[9]*a[k-9] + B[10]*a[k-10] + B[11]*a[k-11] + B[12]*a[99])/A[0];
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

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(249600);
  Serial1.begin(249600);
  Wire.begin();

  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV128_Val;
  //ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT;

  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT),

  //definition of the potentiometer resistance
  potenciometer_resistance();

  //computation of the offset value
  calc_offset();

  //calibration sequence of the acqusition modules
  calibration_sequence(offset);
  
  
  //set all values of the a buffer to zero
  memset(a, 0, n);

  ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, TimerHandler);
}

void loop()
{
  // put your main code here, to run repeatedly:
  

  //When the flag is activated, the RMS value is determined
  
  if(flag)
  {
    digitalWrite(6, HIGH);
    k = 0;
    j = 0;
    static int rmsCounter = 0;
    static float rmsValues[3] = {0.0, 0.0, 0.0};
    float rmsSum = 0;
    float rmsValue;
    
    //The RMS value is calculated
    for(int i = 0; i < n; i++)
    {
      rmsSum += rmsBuffer[i];
      if(i == n - 1)
      {
        rmsValue = sqrt(rmsSum/n);
      }
      else if(i > n - windowWidth/sampleInterval)
      {
        rmsSum -= rmsBuffer[i - n + windowWidth/sampleInterval];
      }
    }

  // send the raw signal, filtered signal and RMS
    /*while(j < n - 1)
    {
      Serial.print(a[j], 2);
      Serial.print(" ");
      Serial.print(f[j], 2);
      Serial.println(" ");
      j++;
    }

    if(j == n - 1)
    {
      Serial.print(a[j]);
      Serial.print(" ");
      Serial.print(f[j]);
      Serial.print(" ");
      Serial.println(rmsValue);
    }*/

  // Send three RMS values at 4 Hz
    rmsValues[rmsCounter] = rmsValue;
    rmsCounter++;

    if (rmsCounter == 3) {
      digitalWrite(7, HIGH);
      // Send the 3 values over serial
      for (int i = 0; i < 3; i++) {
        Serial1.println(rmsValues[i]);
      } // End of batch
      // Reset the counter and timer
      rmsCounter = 0;
      digitalWrite(7, LOW);
    }
    digitalWrite(6, LOW);
    flag = false; 
  }
}
