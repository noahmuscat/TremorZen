#include <Wire.h>
#include <MPU6050.h>

// IMU and Filter Configuration
MPU6050 imu;
float b[] = {0.2929, 0.0000, -0.2929}; // Numerator coefficients from MATLAB
float a[] = {1.0000, -0.5858, 1.0000}; // Denominator coefficients from MATLAB
float x[] = {0, 0, 0}; // Input signal history
float y[] = {0, 0, 0}; // Output signal history

float applyBandpassFilter(float input) {
  x[0] = input;
  y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] - a[1]*y[1] - a[2]*y[2];
  for (int i=2; i>0; i--) {
    x[i] = x[i-1];
    y[i] = y[i-1];
  }
  return y[0];
}

// PID Configuration
float Kp = 1.0, Ki = 0.1, Kd = 0.01; // Replace with optimized parameters from MATLAB
float integral = 0, previousError = 0;
unsigned long lastTime = 0;

float computePID(float setpoint, float measurement) {
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0; // Convert to seconds
  lastTime = now;
  
  float error = setpoint - measurement;
  integral += error * deltaTime;
  float derivative = (error - previousError) / deltaTime;
  previousError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// Actuator Control
int motorPin = 9; // PWM pin connected to the motor driver

void setup() {
  Wire.begin();
  Serial.begin(9600);
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("IMU connection failed!");
  while (1);
  }
  pinMode(motorPin, OUTPUT);
}

void loop() {
 int16_t ax, ay, az;
 imu.getAcceleration(&ax, &ay, &az);
 float accelerometerData = ax / 16384.0; // Convert to g's assuming ±2g range

 // Apply band-pass filter
 float filteredData = applyBandpassFilter(accelerometerData);

 // Compute PID control signal
 float setpoint = 0.0; // Desired value (e.g., stable position)
 float controlSignal = computePID(setpoint, filteredData);

 // Output control signal to actuator
 int pwmSignal = constrain(controlSignal + 128, 0, 255);  // Constrain to 8-bit PWM range
 analogWrite(motorPin, pwmSignal);

 delay(10); // Adjust delay as needed for your sampling rate
 }