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

  AD01.write(0, 20);
  float voltage = (analogRead(A0) * (3.3/1023.0));
  Serial.print("Voltage: ");
  Serial.println(voltage, 3);
  delay(3000);

  AD01.write(0, 30);
  voltage = (analogRead(A0) * (3.3/1023.0));
  Serial.print("Voltage: ");
  Serial.println(voltage, 3);
  delay(3000);

  AD01.write(0, 40);
  voltage = (analogRead(A0) * (3.3/1023.0));
  Serial.print("Voltage: ");
  Serial.println(voltage, 3);
  delay(3000);
}