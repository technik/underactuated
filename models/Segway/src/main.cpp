#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

const int LedPin = 13;

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);

  pinMode(LedPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LedPin, HIGH);
  delay(250);
  digitalWrite(LedPin, LOW);
  delay(500);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}