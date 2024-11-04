#include "arduino_secrets.h"

const int STIMULATION_PINS[] = {9, 10, 11, 12}; // Pins for the four electrodes
const float PULSE_WIDTH = 400;         // in microseconds
const int STIMULATION_CURRENT = 10;    // in mA (you might need to adjust this)
const int STIMULATION_FREQUENCY = 100; // Hz
const unsigned long STIMULATION_DURATION = 2000; // Stimulation duration in milliseconds

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Set stimulation pins as output
  for (int i = 0; i < 4; i++) {
    pinMode(STIMULATION_PINS[i], OUTPUT);
  }
  Serial.println("Stimulation Setup Complete.");
}

void loop() {
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == 'S') { // Start stimulation if 'S' is received via Serial
      Serial.println("Starting Stimulation...");
      applyStimulus();
      Serial.println("Stimulation Complete.");
    }
  }
}

void applyPulse(int pin) {
  int pwmValue = map(STIMULATION_CURRENT, 0, 20, 0, 255);
  analogWrite(pin, pwmValue); // Generate a PWM signal for the specified pulse width
  delayMicroseconds(PULSE_WIDTH);
  analogWrite(pin, 0); // Turn off the pin
  Serial.print("Applied Pulse to Pin ");
  Serial.print(pin);
  Serial.print(" with PWM value ");
  Serial.println(pwmValue);
}

void applyStimulus() {
  // Apply stimulation pulses for the duration of the stimulation phase
  unsigned long startTime = millis();
  while (millis() - startTime < STIMULATION_DURATION) {
    for (int i = 0; i < 4; i++) {
      applyPulse(STIMULATION_PINS[i]);
    }
    delay(1000 / STIMULATION_FREQUENCY); // Delay to achieve desired frequency
  }
}
