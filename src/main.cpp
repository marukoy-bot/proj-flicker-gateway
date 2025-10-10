#include <Arduino.h>

void setup () {
    pinMode(2, INPUT);
    pinMode(13, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    Serial.println(digitalRead(2));
    digitalWrite(13, digitalRead(2));
    delay(100);
}