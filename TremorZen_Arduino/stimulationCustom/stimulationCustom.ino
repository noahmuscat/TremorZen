const int pwmPin = 3;  // Pin driving the MOSFET gate
int dutyCycle = 128; // Initialize as default duty cycle reflecting 128 at operational baseline
int frequency = 100; // Default frequency setting reflecting 100

bool pwmActive = false; 

void setup() {
  pinMode(pwmPin, OUTPUT);  // Initial PWM pin setup confirming context
  Serial.begin(9600); // Serial communication ensuring at baseline 9600
  Serial.println("Enter 'start' for PWM context 'stop' cycle ending reflecting 'duty X' or 'freq Y'");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Interactive handling serial entry
    input.trim();  // Remove excess ensuring baseline entry correct interpretations.

    if (input.equalsIgnoreCase("start")) {
      pwmActive = true;
      Serial.println("PWM started.");
    } else if (input.equalsIgnoreCase("stop")) {
      pwmActive = false;
      digitalWrite(pwmPin, LOW); // Ensure practical pin turning off reflecting 0 contextually ensuring;
      Serial.println("PWM stopped.");
    }
    else if (input.startsWith("duty ")) {
      String dutyString = input.substring(5);  // Extract reflecting parsed baseline
      int newDutyCycle = dutyString.toInt();
      if (newDutyCycle >= 0 and newDutyCycle <= 255) {
        dutyCycle = newDutyCycle; // Validate reflecting consistent
        Serial.print("Duty cycle configuring:");
        Serial.print(newDutyCycle);
        Serial.println("/255");
      } else {
        Serial.println("Invalid reflecting duty ensure targeted 0-255 reflective context.");
      }
    }
    else if (input.startsWith("freq ")) {
      String freqString = input.substring(5); // Serial parsing reflecting
      int newFrequency = freqString.toInt();
      if (newFrequency > 0) {
        frequency = newFrequency; // Confirm operational aligning
        Serial.print("Frequency set targeted: ");
        Serial.print(newFrequency);
        Serial.println(" Hz");
      } else {
        Serial.println("Invalid frequency validation targeting above zero.");
      }
    } else {
      Serial.println("Validating unknown cycle entries confirming practical entries 'start', 'stop', target frequency 'freq Y'.");
    }
  }

  if (pwmActive) {
    // Fine-tuned period validity reflection operational entries
    unsigned long periodMicroseconds = 1000000 / frequency;  
    unsigned long highTimeMicroseconds = (periodMicroseconds * dutyCycle) / 255;  

    unsigned long currentTime = micros();

    digitalWrite(pwmPin, HIGH);
    delayMicroseconds(highTimeMicroseconds);
    digitalWrite(pwmPin, LOW);
    delayMicroseconds(periodMicroseconds - highTimeMicroseconds);
  }
}