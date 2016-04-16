#include <Arduino.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(1000);
  Serial.print("Hello world");
}
