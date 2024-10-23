% written in C++ here, but in Matlab lol. 
% I uninstalled my C++ IDE awhile ago so this will do until we actually get the Arduino

// necessary libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h> // Replace with your specific IMU library


#define OUTPUT_PIN 9 // Pin connected to the electrical stimulator
#define THRESHOLD 0.1 // Detection threshold
#define STIMULATION_DURATION 10000 // 10 seconds in milliseconds

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // replace with your IMU setup

void setup() {
  Serial.begin(9600);
  if (!accel.begin()) {
    Serial.println("No ADXL345 detected ... Check your wiring!");
    while (1);
  }
  pinMode(OUTPUT_PIN, OUTPUT);
}

void loop() {
  sensors_event_t event;
  accel.getEvent(&event);

  double filteredSignal = bandPassFilter(event.acceleration.x); // Apply filter to the X-axis data

  if (detectTremor(filteredSignal)) {
    stimulateMuscle();
  }

  delay(10); // Control loop speed, adjust as needed to match sampling frequency
}

double bandPassFilter(double input) {
  static double prevInput[2] = {0.0, 0.0};
  static double prevOutput[2] = {0.0, 0.0};

  const double a[] = {1.0, -1.5610180758007182, 0.6413515380575631}; // Coefficients for the band-pass filter
  const double b[] = {0.020083365564211235, 0.0, -0.020083365564211235};

  double output = (input * b[0]) + (prevInput[0] * b[1]) + (prevInput[1] * b[2])
                - (prevOutput[0] * a[1]) - (prevOutput[1] * a[2]);

  prevInput[1] = prevInput[0];
  prevInput[0] = input;
  prevOutput[1] = prevOutput[0];
  prevOutput[0] = output;

  return output;
}

bool detectTremor(double signal) {
  return abs(signal) > THRESHOLD;
}

void stimulateMuscle() {
  digitalWrite(OUTPUT_PIN, HIGH); // Turn on the electrical stimulator
  delay(STIMULATION_DURATION); // Stimulate for 10 seconds
  digitalWrite(OUTPUT_PIN, LOW); // Turn off the electrical stimulator
}