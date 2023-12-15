#include <IRremote.h>

const int irReceiverPin = 2;  // Connect the IR receiver signal pin to Arduino digital pin 2

IRrecv irrecv(irReceiverPin);
decode_results results;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();  // Start the IR receiver
}

void loop() {
  if (irrecv.decode(&results)) {
    // Check if the received signal is close to 550Hz
    if (results.decode_type == UNKNOWN && results.rawlen > 0) {
      unsigned int frequency = 1000000 / (results.rawbuf[1] * USECPERTICK);  // Calculate frequency in Hz
      Serial.print("Received signal at ");
      Serial.print(frequency);
      Serial.println(" Hz");

      // Check if the frequency is close to 550Hz (adjust the tolerance as needed)
      if (abs(frequency - 550) < 10) {
        Serial.println("Detected 550Hz IR signal!");
        // Add your code here to perform an action when the signal is detected
      }
    }

    irrecv.resume();  // Receive the next value
  }
}
