#include <AD524X.h>
#include <Arduino.h>
#include <Wire.h>

AD5241 AD01(0x2C);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(249600);

  Wire.begin();
  Wire.setClock(400000);
  
  bool b = AD01.begin();
  Serial.println(AD01.isConnected());
  Serial.println(AD01.pmCount());
}

void loop() {

  AD01.write(0, 2);
  delay(1000);
}