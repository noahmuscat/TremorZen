#include <Arduino_LSM9DS1.h>
#include "Arduino_BMI270_BMM150.h"
#include <arm_math.h>

// Define filter order and coefficients
#define FILTER_ORDER 4

// Optimized Band Pass Filter coefficients
float b[] = {0.00080636, 0, -0.00322544, 0, 0.00483816, 0, -0.00322544, 0, 0.00080636};
float a[] = {1, -6.03196, 16.7302, -27.7277, 29.9741, -21.6263, 10.1777, -2.86301, 0.370814};

// Initialize IIR filter structures
arm_biquad_casd_df1_inst_f32 Sx, Sy, Sz;
float stateBx[4 * FILTER_ORDER];
float stateBy[4 * FILTER_ORDER];
float stateBz[4 * FILTER_ORDER];

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
  arm_biquad_cascade_df1_init_f32(&Sx, FILTER_ORDER / 2, b, stateBx);
  arm_biquad_cascade_df1_init_f32(&Sy, FILTER_ORDER / 2, b, stateBy);
  arm_biquad_cascade_df1_init_f32(&Sz, FILTER_ORDER / 2, b, stateBz);
}

void loop() {
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);

    // Print raw IMU data for analysis
    //Serial.print("RAW IMU: ");
    //Serial.print(ax); Serial.print(" ");
    //Serial.print(ay); Serial.print(" ");
    //Serial.println(az);

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
    //Serial.print("Filtered IMU data: ");
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