#include <WiFi.h>

#include "Webpage.h"
#include "html510.h"

#define TRIG_PIN     4                // GPIO pin for triger on ultrasonic sensor
#define ECHO_PIN     5              // GPIO pin for echo 
#define Motor_channel1 0 
#define Motor_channel2 1 
#define Motor_channel3 2
const int HeadPin = 1;  // GPIO pin for the Head Motor
const int LeftMotor = 0; //// GPIO pin for the LeftMotor

#define Motor_freq 50  //Motor frequency
#define Motor_resolution_bits 12 //Motor resolution in bits
#define Motor_resolution ((1<<Motor_resolution_bits)-1)

unsigned int distance;
unsigned int FrontDistance;
unsigned int LeftDistance;
unsigned int RightDistance;
unsigned int LeftDiagonalDistance;
unsigned int RightDiagonalDistance;

HTML510Server h(80);
WiFiServer server(80);
char numberArray[20];

const char* ssid = "TP-Link_E0C8";  //TP-Link_E0C8
const char* password = "52665134";   //52665134

char choice;
char turnDirection;  // Gets 'l', 'r' or 'f' depending on which direction is obstacle free

int distanceCounter = 0;
int numcycles = 0;  // Number of cycles used to rotate with head during moving
int roam = 0;       // Switching between automatic and manual mode of moving
 
// limits for obstacles:
const int distanceLimit = 27;           // Front distance limit in cm
const int sideDistanceLimit = 12;       // Side distance limit in cm
const int turnTime = 300;               // Time needed to turn robot

void setup(){
  Serial.begin(9600);  
  ledcSetup(Motor_channel1, Motor_freq, Motor_resolution_bits); // setup Head Motor channel1
  ledcAttachPin(HeadPin, Motor_channel1);
  ledcSetup(Motor_channel2, Motor_freq, Motor_resolution_bits); // setup Left Motor channel2
  ledcAttachPin(LeftMotor, Motor_channel2);

  // ultrasonic pin set
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  IPAddress myIP(192,168,1,130);  //change to your own IP address
  IPAddress routerIP(192,168,1,1); //192,168,1,1
  
  WiFi.mode(WIFI_MODE_STA);    //static mode
  WiFi.begin(ssid, password);
  WiFi.config(myIP,routerIP,IPAddress(255,255,255,0));
  
  while(WiFi.status()!= WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  } //check WiFi conncetion
  Serial.printf("connected to %s on",ssid); 
  Serial.println(myIP);

  h.begin();   //start the server
  h.attachHandler("/",handleRoot);
  h.attachHandler("/setDirection?val=", handleSlider1);    //attach sliders
  h.attachHandler("/setMode?val=", handleSlider2);
  h.attachHandler("/setLeft", handleButton1);       //attach buttons
  h.attachHandler("/setRight", handleButton2);
  
  moveStop();
}

void handleRoot(){
  h.sendhtml(body);
}

void handleSlider1() {
  int sliderValue = h.getVal();
  String s = "Move";
  if (sliderValue == 1) {
    moveBackward();
    s = s+ " Backward";
    Serial.println(sliderValue);
  } else if(sliderValue == 3) {
    moveForward();
    s = s+ " Forward";
    Serial.println(sliderValue);
  } else if(sliderValue == 2) {
    moveStop();
    s = s+ " Stop";
    Serial.println(sliderValue);
  }
  h.sendplain(s);
} // mode slider: move forward, stop, or backward

void handleSlider2() {
  int sliderValue = h.getVal();
  String s = "";
  if (sliderValue == 1){
    Serial.println("Automatic mode");
    moveStop();
    toggleRoam(); 
    s = s+ "Automatic";
  } else if(sliderValue == 0) {
    Serial.println("Control mode");
    moveStop();
    s = s+ "Control";
    Serial.println(sliderValue);
  }
  h.sendplain(s);
} // Mode slider: control/autonomous

void handleButton1() {
  moveLeft();
  delay(turnTime);
  h.sendplain("");
} //Button Left

void handleButton2() {
  moveRight();
  delay(turnTime);
  h.sendplain("");
} //Button Right

//This function determines the distance things are away from the ultrasonic sensor
void scan(){
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

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 4095){
  uint32_t duty = Motor_resolution * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty); // write duty to Motor pin
}

//This function toggle between autonomous and stop mode
void toggleRoam(){
  if(roam == 0){
    roam = 1;
    Serial.println("Activated Roam Mode");
  }
  else{
    roam = 0;
    moveStop();
    Serial.println("De-activated Roam Mode");
  }
}

//This function tells the robot to go forward 
void moveForward(){
  Serial.println("");
  Serial.println("Moving forward");
  ledcAnalogWrite(Motor_channel2,map(0, 0, 180, 122, 492));
  //ledcAnalogWrite(Motor_channel3,map(180, 0, 180, 122, 492));
}

//This function tells the robot to move backward
void moveBackward(){
  Serial.println("");
  Serial.println("Moving backward");
  ledcAnalogWrite(Motor_channel2,map(180, 0, 180, 122, 492));
  //ledcAnalogWrite(Motor_channel3,map(0, 0, 180, 122, 492));
}

//This function tells the robot to turn left
void moveRight(){
  Serial.println("");
  Serial.println("Moving left");
  ledcAnalogWrite(Motor_channel2,map(60, 0, 180, 122, 492));
  //ledcAnalogWrite(Motor_channel3,map(90, 0, 180, 122, 492));
}

//This function tells the robot to turn right
void moveLeft(){
  Serial.println("");
  Serial.println("Moving right");
  ledcAnalogWrite(Motor_channel2,map(120, 0, 180, 122, 492));
  //ledcAnalogWrite(Motor_channel3,map(120, 0, 180, 122, 492));
}

//This function tells the robot to stop moving
void moveStop(){
  Serial.println("");
  Serial.println("Stopping");
  ledcAnalogWrite(Motor_channel2,map(90, 0, 180, 122, 492));
  //ledcAnalogWrite(Motor_channel3,map(90, 0, 180, 122, 492));
}

void watchsurrounding(){ 
  //Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
  //leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
  // Scanning front 
  scan();
  FrontDistance = distance;
  Serial.println("Front distance measuring done");
  if(FrontDistance < distanceLimit){
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(130, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scanning left diagnal
  scan();
  LeftDiagonalDistance = distance;
  Serial.println("Left diagonal distance measuring done");
  if(LeftDiagonalDistance < distanceLimit){
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(162, 0, 180, 122, 492));  // update Motor duty cycle
  delay(300);

  // Scanning left
  scan();
  LeftDistance = distance;
  Serial.println("Left distance measuring done");
  if(LeftDistance < sideDistanceLimit){
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(130, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scanning left diagnal
  scan();
  LeftDiagonalDistance = distance;
  Serial.println("Left diagonal distance measuring done");
  if(LeftDiagonalDistance < distanceLimit){
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(98, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scanning front
  scan();
  FrontDistance = distance;
  Serial.println("Front distance measuring done");
  if(FrontDistance < distanceLimit){
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(66, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scannign right diagnal
  scan();
  RightDiagonalDistance = distance;
  Serial.println("Right diagonal distance measuring done");
  if(RightDiagonalDistance < distanceLimit){
    moveStop();
  }
  ledcAnalogWrite(Motor_channel1,map(34, 0, 180, 122, 492));  // update Motor duty cycle
  delay(100);

  // Scanning right
  scan();
  RightDistance = distance;
  Serial.println("Right distance measuring done");
  if(RightDistance < sideDistanceLimit){
    moveStop();
  }

  //Finish looking around (look forward again)
  ledcAnalogWrite(Motor_channel1,map(98, 0, 180, 122, 492));  // update Motor duty cycle
  delay(300);
  Serial.println("Measuring done");
}

char decide(){
   // Decide the right way without obstacles
  watchsurrounding();
  if (LeftDistance > RightDistance && LeftDistance > FrontDistance){
    Serial.println("Choise result is: LEFT");
    choice = 'l';
  }
  else if (RightDistance > LeftDistance && RightDistance > FrontDistance){
    Serial.println("Choise result is: RIGHT");
    choice = 'r';
  }
  else if ( LeftDistance < sideDistanceLimit && RightDistance < sideDistanceLimit && FrontDistance < distanceLimit ) {
    Serial.println("Choice result is: BACK"); 
    choice = 'b';
  }
  else{
    Serial.println("Choise result is: FORWARD");
    choice = 'f';
  }
  return choice;
}

void go() {
  moveForward();
  ++numcycles;
  // After 40 cycles of code measure surrounding obstacles
  if(numcycles>40){
    Serial.println("Front obstancle detected");
    watchsurrounding();
    if( LeftDistance < sideDistanceLimit || LeftDiagonalDistance < sideDistanceLimit){
      Serial.println("Moving: RIGHT");
      moveRight();
      delay(turnTime);
    }
    if( RightDistance < sideDistanceLimit || RightDiagonalDistance < sideDistanceLimit){
      Serial.println("Moving: LEFT");
      moveLeft();
      delay(turnTime);
    }
    numcycles=0; //Restart count of cycles
  }
  scan();
  if( distance < distanceLimit){
    distanceCounter++;
  }
  if( distance > distanceLimit){
    distanceCounter = 0;
  }
  // robot reachaed 7 times distance limit in front of the robot, so robot must stop immediately and decide right way
  if(distanceCounter > 7){
    moveStop();
    turnDirection = decide();
     switch (turnDirection){
      case 'l':
        moveLeft();
        delay(turnTime);
        break;
      case 'r':
        moveRight();
        delay(turnTime);
        break;
      case 'b':
        moveBackward();
        delay(turnTime);
        break;
      case 'f':
        break;
    }
    distanceCounter = 0;
  }
}

void loop(){
  if(roam == 1){
    go();
  }
}
