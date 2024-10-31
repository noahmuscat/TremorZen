#include <Arduino_LSM9DS1.h>
#include <arm_math.h>

// Define filter order and coefficients
#define FILTER_ORDER 4

// Optimized Band Pass Filter coefficients
float b[] = {0.00080636, 0, -0.00322544, 0, 0.00483816, 0, -0.00322544, 0, 0.00080636, };
float a[] = {1, -6.03196, 16.7302, -27.7277, 29.9741, -21.6263, 10.1777, -2.86301, 0.370814, };

// Initialize IIR filter structures for "B" and "A" coefficients
arm_biquad_casd_df1_inst_f32 Sx, Sy, Sz;
float stateBx[4 * FILTER_ORDER];
float stateBy[4 * FILTER_ORDER];
float stateBz[4 * FILTER_ORDER];

// Control system timing constants
const unsigned long RECORDING_DURATION = 1000;    // Record for 1 second
const unsigned long STIMULATION_DURATION = 2000;  // Stimulate for 2 seconds

// Variables to track timing and system state
unsigned long previousMillis = 0;   // Stores the last time the state was changed
bool isRecordingPhase = true;       // Tracks whether we are in recording or stimulation phase

// Variables for tremor detection
bool tremorDetected = false;
unsigned long tremorDetectionTime = 0;   // Time when tremor is detected

// Stimulation parameters
const int STIMULATION_PINS[] = {9, 10, 11, 12}; // Pins for the four electrodes
const float PULSE_WIDTH = 400;         // in microseconds
const int STIMULATION_CURRENT = 10;    // in mA (you might need to adjust this)
const int STIMULATION_FREQUENCY = 100; // Hz

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
  unsigned long currentMillis = millis();
  
  if (isRecordingPhase) {
    // Check if it's time to switch to stimulation phase
    if (currentMillis - previousMillis >= RECORDING_DURATION) {
      previousMillis = currentMillis;
      isRecordingPhase = false; // Switch to stimulation phase
    }

    // Perform recording phase tasks
    if (IMU.accelerationAvailable()) {
      float ax, ay, az;
      IMU.readAcceleration(ax, ay, az);

      // Apply the band-pass filter to the x, y, and z-axis data
      float filtered_value_x, filtered_value_y, filtered_value_z;
      arm_biquad_cascade_df1_f32(&Sx, &ax, &filtered_value_x, 1);
      arm_biquad_cascade_df1_f32(&Sy, &ay, &filtered_value_y, 1);
      arm_biquad_cascade_df1_f32(&Sz, &az, &filtered_value_z, 1);

      // Combine filtered values
      float combined_filtered_value = sqrt(filtered_value_x * filtered_value_x +
                                           filtered_value_y * filtered_value_y +
                                           filtered_value_z * filtered_value_z);

      // Store or process data if needed
      Serial.print("Recorded data: ");
      Serial.println(combined_filtered_value);

      // Tremor detection
      if (detectTremor(combined_filtered_value)) {
        tremorDetected = true;
        tremorDetectionTime = currentMillis;
        Serial.println("Tremor Detected!");
      }
    }

  } else { // Stimulation phase
    // Check if it's time to switch back to recording phase
    if (currentMillis - previousMillis >= STIMULATION_DURATION) {
      previousMillis = currentMillis;
      isRecordingPhase = true;  // Switch to recording phase
      return; // Ensuring that no more stimulation happens in the same loop pass
    }

    // Perform stimulation phase tasks
    if (tremorDetected) {
      // Calculate time elapsed since tremor was detected
      unsigned long timeSinceDetection = currentMillis - tremorDetectionTime;
      // Handle stimulation timing here, for simplicity, immediately stimulate
      applyStimulus();
      tremorDetected = false; // Reset tremor detection flag
    }
  }
}

bool detectTremor(float filtered_value) {
  const float TREMOR_THRESHOLD = 1.0; // Define an appropriate threshold for tremor
  return (abs(filtered_value) > TREMOR_THRESHOLD);
}

void applyPulse(int pin) {
  // Generate a PWM signal for the specified pulse width
  analogWrite(pin, map(STIMULATION_CURRENT, 0, 20, 0, 255)); // Map current to PWM range
  delayMicroseconds(PULSE_WIDTH);
  analogWrite(pin, 0); // Turn off the pin
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