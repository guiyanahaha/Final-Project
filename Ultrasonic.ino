//  * Ultrasonic sensor TRIG PIN: D09
//  * Ultrasonic sensor ECHO PIN: D10

#define TRIG_PIN     4                // GPIO pin for triger on ultrasonic sensor
#define ECHO_PIN     5              // GPIO pin for echo 
#define Motor_channel1 0 
#define Motor_channel2 1 
#define Motor_channel3 2
const int HeadPin = 1;  // GPIO pin for the Head Motor

#define Motor_freq 50  //Motor frequency
#define Motor_resolution_bits 12 //Motor resolution in bits
#define Motor_resolution ((1<<Motor_resolution_bits)-1)

unsigned int distance;
unsigned int FrontDistance;
unsigned int LeftDistance;
unsigned int RightDistance;
unsigned int LeftDiagonalDistance;
unsigned int RightDiagonalDistance;
 
// limits for obstacles:
const int distanceLimit = 27;           // Front distance limit in cm
const int sideDistanceLimit = 12;       // Side distance limit in cm
const int turnTime = 300;               // Time needed to turn robot

long duration;

void setup()                                        
{
  Serial.begin(9600);  
  ledcSetup(Motor_channel1, Motor_freq, Motor_resolution_bits); // setup Motor1 channel1
  ledcAttachPin(HeadPin, Motor_channel1); //motor 1    

  // ultrasonic pin set
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void scan()                                         //This function determines the distance things are away from the ultrasonic sensor
{
  long pulse;
  Serial.println("Scanning distance");
  digitalWrite(TRIG_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(TRIG_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN,LOW);
  pulse = pulseIn(ECHO_PIN,HIGH);
  distance = round( pulse*0.01657 );
  Serial.println(distance);
}

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 4095) 
{
  uint32_t duty = Motor_resolution * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty); // write duty to Motor pin
}


void moveStop()
{

}

void watchsurrounding()
{ //Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
  //leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
  // Scanning front 
  scan();
  FrontDistance = distance;
  Serial.println("Front distance measuring done");
  if(FrontDistance < distanceLimit) 
  {
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(130, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scanning left diagnal
  scan();
  LeftDiagonalDistance = distance;
  Serial.println("Left diagonal distance measuring done");
  if(LeftDiagonalDistance < distanceLimit)
  {
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(162, 0, 180, 122, 492));  // update Motor duty cycle
  delay(300);

  // Scanning left
  scan();
  LeftDistance = distance;
  Serial.println("Left distance measuring done");
  if(LeftDistance < sideDistanceLimit)
  {
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(130, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scanning left diagnal
  scan();
  LeftDiagonalDistance = distance;
  Serial.println("Left diagonal distance measuring done");
  if(LeftDiagonalDistance < distanceLimit)
  {
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(98, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scanning front
  scan();
  FrontDistance = distance;
  Serial.println("Front distance measuring done");
  if(FrontDistance < distanceLimit)
  {
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(66, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scannign right diagnal
  scan();
  RightDiagonalDistance = distance;
  Serial.println("Right diagonal distance measuring done");
  if(RightDiagonalDistance < distanceLimit)
  {
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(34, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scanning right
  scan();
  RightDistance = distance;
  Serial.println("Right distance measuring done");
  if(RightDistance < sideDistanceLimit)
  {
    moveStop();
  }

  //Finish looking around (look forward again)
  ledcAnalogWrite(Motor_channel1,map(98, 0, 180, 122, 492));  // update Motor duty cycle
  delay(300);
  Serial.println("Measuring done");
}


void loop()
{
  watchsurrounding();
}
