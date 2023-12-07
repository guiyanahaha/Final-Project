const int PHOTOTRANSISTOR_PIN = 10;
const int LED_RED = 19;
const int LED_GREEN = 18;

static int old_time = 0;
int frequency = 0;
volatile uint32_t Down_time = 0;

void IRAM_ATTR handleInterrupt1() {
  Down_time = millis(); // millis returns ms since the program started.
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(PHOTOTRANSISTOR_PIN, INPUT);  // Set the phototransistor pin as input
  attachInterrupt(digitalPinToInterrupt(PHOTOTRANSISTOR_PIN), handleInterrupt1, FALLING);
}

void loop() {
  if (old_time != Down_time){
      frequency = Down_time - old_time;
      old_time = Down_time;
  }
  if (frequency >= 21 && frequency <= 25) {
        digitalWrite(LED_RED, HIGH); // Turn on the red LED
        digitalWrite(LED_GREEN, LOW); // Turn off the green LED
        Serial.print("Frequency: ");
        Serial.println(frequency);
    } else if (frequency >= 540 && frequency <= 560){
        digitalWrite(LED_RED, LOW); // Turn off the red LED
        digitalWrite(LED_GREEN, HIGH);  // Turn on the green LED
        Serial.print("Frequency: ");
        Serial.println(frequency);
    }else {
        digitalWrite(LED_RED, LOW); // Turn off all LED
        digitalWrite(LED_GREEN, LOW);
        Serial.print("Frequency: ");
        Serial.println(frequency);
    }
  
}
