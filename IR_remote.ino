const float a = 0.239; // Updated filter coefficient for 50 Hz cutoff
float filteredValue = 0; // Holds the filtered value
int lastReading = 0; // Holds the last reading
unsigned long lastCrossingTime = 0; // Time of the last zero crossing
bool crossingDetected = false; // Flag to avoid multiple detections for the same crossing

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
}

void loop() {
  // Read the analog value (example)
  int sensorValue = analogRead(0); 

  // Convert to a voltage, assuming a 3.3V system and 12-bit ADC resolution
  float voltage = sensorValue * (3.3 / 4095.0);

  // Apply the low-pass filter with the new coefficient
  filteredValue = a * voltage + (1 - a) * filteredValue;

  // Check for zero crossing
  if ((lastReading <= 0 && filteredValue > 0) || (lastReading >= 0 && filteredValue < 0)) {
    if (!crossingDetected) {
      unsigned long currentTime = millis();
      if (lastCrossingTime > 0) { // Skip the first crossing
        unsigned long period = currentTime - lastCrossingTime;
        if (period > 0) { // Avoid division by zero
          float frequency = 1000.0 / period; // Calculate frequency
          Serial.print("Frequency: ");
          Serial.println(frequency);
        }
      }
      lastCrossingTime = currentTime;
      crossingDetected = true;
    }
  } else {
    crossingDetected = false;
  }

  lastReading = filteredValue;

  // Delay to match the sampling frequency (1ms for 1000Hz)
  delay(1);
}
