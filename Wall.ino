#define Motor_channel1 0 
#define Motor_channel2 1 
#define Motor_channel3 2

#define Motor_freq 50  //Motor frequency
#define Motor_resolution_bits 12 //Motor resolution in bits
#define Motor_resolution ((1<<Motor_resolution_bits)-1)

const int HeadPin = 1;  // GPIO pin for the Head Motor

void setup() {
  Serial.begin(115200);
  
  ledcSetup(Motor_channel1, Motor_freq, Motor_resolution_bits); // setup Motor1 channel1
  ledcAttachPin(HeadPin, Motor_channel1); //motor 1
}

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 4095) {
  uint32_t duty = Motor_resolution * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty); // write duty to Motor pin
}

void loop() {
    ledcAnalogWrite(Motor_channel1,map(130, 0, 180, 122, 492));  // update Motor duty cycle
    delay(100); 

    ledcAnalogWrite(Motor_channel1,map(162, 0, 180, 122, 492));  // update Motor duty cycle
    delay(300);

    ledcAnalogWrite(Motor_channel1,map(130, 0, 180, 122, 492));  // update Motor duty cycle
    delay(100);

    ledcAnalogWrite(Motor_channel1,map(98, 0, 180, 122, 492));  // update Motor duty cycle
    delay(100);      

    ledcAnalogWrite(Motor_channel1,map(66, 0, 180, 122, 492));  // update Motor duty cycle
    delay(100);
    
    ledcAnalogWrite(Motor_channel1,map(34, 0, 180, 122, 492));  // update Motor duty cycle
    delay(100);

    ledcAnalogWrite(Motor_channel1,map(98, 0, 180, 122, 492));  // update Motor duty cycle
    delay(300); 
}
