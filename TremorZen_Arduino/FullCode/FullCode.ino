#include "arduino_secrets.h"

#include <Arduino_LSM9DS1.h>
#include <arm_math.h>

// Define filter order and coefficients
#define FILTER_ORDER 4

// Optimized Band Pass Filter coefficients
float b[] = {0.00080636, 0, -0.00322544, 0, 0.00483816, 0, -0.00322544, 0, 0.00080636 };
float a[] = {1, -6.03196, 16.7302, -27.7277, 29.9741, -21.6263, 10.1777, -2.86301, 0.370814 };

// Initialize IIR filter structures for "B" and "A" coefficients
arm_biquad_casd_df1_inst_f32 Sx, Sy, Sz;
float stateBx[4 * FILTER_ORDER];
float stateBy[4 * FILTER_ORDER];
float stateBz[4 * FILTER_ORDER];

// Stimulation parameters and variables
const int STIMULATION_PINS[] = {9, 10, 11, 12}; // Pins for the four electrodes
const float PULSE_WIDTH = 400;         // in microseconds
const int STIMULATION_CURRENT = 10;    // in mA (you might need to adjust this)
const int STIMULATION_FREQUENCY = 100; // Hz
const unsigned long STIMULATION_DURATION = 2000;  // Stimulate for 2 seconds

bool tremorDetected = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU Initialized.");

  // Initialize Biquad cascade filter with coefficients for each axis
  arm_biquad_cascade_df1_init_f32(&Sx, FILTER_ORDER / 2, b, stateBx);
  arm_biquad_cascade_df1_init_f32(&Sy, FILTER_ORDER / 2, b, stateBy);
  arm_biquad_cascade_df1_init_f32(&Sz, FILTER_ORDER / 2, b, stateBz);

  // Set stimulation pins as output
  for (int i = 0; i < 4; i++) {
    pinMode(STIMULATION_PINS[i], OUTPUT);
  }
}

void loop() {
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);

    // Print raw IMU data for analysis
    Serial.print("RAW IMU: ");
    Serial.print(ax); 
    Serial.print(" ");
    Serial.print(ay); 
    Serial.print(" ");
    Serial.println(az);

    // Apply the band-pass filter to the x, y, and z-axis data
    float filtered_value_x, filtered_value_y, filtered_value_z;
    arm_biquad_cascade_df1_f32(&Sx, &ax, &filtered_value_x, 1);
    arm_biquad_cascade_df1_f32(&Sy, &ay, &filtered_value_y, 1);
    arm_biquad_cascade_df1_f32(&Sz, &az, &filtered_value_z, 1);

    // Combine filtered values
    float combined_filtered_value = sqrt(filtered_value_x * filtered_value_x +
                                         filtered_value_y * filtered_value_y +
                                         filtered_value_z * filtered_value_z);

    // Print filtered data for analysis
    Serial.print("Filtered data: ");
    Serial.println(combined_filtered_value);

    // Tremor detection
    if (detectTremor(combined_filtered_value)) {
      Serial.println("Tremor Detected! Applying Stimulation...");
      applyStimulus();
      Serial.println("Stimulation Complete.");
    }
  }
}

bool detectTremor(float filtered_value) {
  const float TREMOR_THRESHOLD = 1.0; // Define an appropriate threshold for tremor
  return (abs(filtered_value) > TREMOR_THRESHOLD);
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
