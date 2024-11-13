#include <Arduino_LSM9DS1.h>
#include "Arduino_BMI270_BMM150.h"
#include <arm_math.h>

// Define filter order and coefficients
#define FILTER_ORDER 4  // Note: FILTER_ORDER should be twice the number of biquad stages, so this implies 2 stages

// Optimized Band Pass Filter coefficients (adjusted for biquad)
float biquad_coeffs[] = {
  0.00080636,  0.00000000, -0.00080636,  1.92741942, -0.93533400,  // Section 1
  1.00000000, -1.53895826,  0.71794646, 1.92741942, -0.93533400   // Section 2
};

// Initialize IIR filter structures
arm_biquad_casd_df1_inst_f32 Sx, Sy, Sz;
float stateBx[4 * (FILTER_ORDER / 2)];
float stateBy[4 * (FILTER_ORDER / 2)];
float stateBz[4 * (FILTER_ORDER / 2)];

// Variables for tremor detection and control
bool tremorDetected = false;
unsigned long tremorDetectionTime = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  // Add a small delay before initializing IMU
  delay(500);

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);  // Stay here if initialization failed
  }
  Serial.println("IMU Initialized.");

  // Initialize Biquad cascade filter for each axis
  arm_biquad_cascade_df1_init_f32(&Sx, FILTER_ORDER / 2, biquad_coeffs, stateBx);
  arm_biquad_cascade_df1_init_f32(&Sy, FILTER_ORDER / 2, biquad_coeffs, stateBy);
  arm_biquad_cascade_df1_init_f32(&Sz, FILTER_ORDER / 2, biquad_coeffs, stateBz);
}

void loop() {
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);

    // Apply band-pass filter to x, y, and z-axis data
    float filtered_value_x, filtered_value_y, filtered_value_z;
    arm_biquad_cascade_df1_f32(&Sx, &ax, &filtered_value_x, 1);
    arm_biquad_cascade_df1_f32(&Sy, &ay, &filtered_value_y, 1);
    arm_biquad_cascade_df1_f32(&Sz, &az, &filtered_value_z, 1);

    // Combine filtered values
    float combined_filtered_value = sqrt(filtered_value_x * filtered_value_x +
                                         filtered_value_y * filtered_value_y +
                                         filtered_value_z * filtered_value_z);

    unsigned long currentMicros = micros();
    
    // Print filtered IMU data for analysis
    Serial.print(currentMicros); Serial.print(" ");
    Serial.print(ax); Serial.print(" ");
    Serial.print(ay); Serial.print(" ");
    Serial.print(az); Serial.print(" ");
    Serial.print(filtered_value_x); Serial.print(" ");
    Serial.print(filtered_value_y); Serial.print(" ");
    Serial.print(filtered_value_z); Serial.print(" ");
    Serial.println(combined_filtered_value);

    // Tremor detection
    if (detectTremor(combined_filtered_value)) {
      tremorDetected = true;
      tremorDetectionTime = millis();
      Serial.println("Tremor Detected!");
    }
  }
}

bool detectTremor(float filtered_value) {
  const float TREMOR_THRESHOLD = 1.0; // Define an appropriate threshold for tremor
  return (abs(filtered_value) > TREMOR_THRESHOLD);
}