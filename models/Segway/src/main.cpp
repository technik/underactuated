#include <Arduino.h>

#include <CAN.h>

const int LedPin = 13;

void setup() {
  pinMode(LedPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LedPin, HIGH);
  delay(250);
  digitalWrite(LedPin, LOW);
  delay(500);
}