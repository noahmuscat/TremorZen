const int STIMULATION_PIN = 9; // Pin to be tested

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Set the stimulation pin as output
  pinMode(STIMULATION_PIN, OUTPUT);

  Serial.println("Stimulation Setup Complete.");
}

void loop() {
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == 'S') { // Start stimulation if 'S' is received via Serial
      Serial.println("Starting Stimulation...");
      digitalWrite(STIMULATION_PIN, HIGH); // Turn on the pin to output high voltage
      delay(10000); // Keep the pin HIGH for 10 seconds to measure the current
      digitalWrite(STIMULATION_PIN, LOW); // Turn off the pin
      Serial.println("Stimulation Complete. Pin is LOW.");
    }
  }
}