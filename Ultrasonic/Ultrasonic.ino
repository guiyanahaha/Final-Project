#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>

#include "vive510.h"
#include "Webpage.h"
#include "html510.h"

#define TRIG_PIN     4                // GPIO pin for triger on ultrasonic sensor
#define ECHO_PIN     5              // GPIO pin for echo 
#define Motor_channel1 0 
#define Motor_channel2 1 
#define Motor_channel3 2
const int HeadPin = 1;  // GPIO pin for the Head Motor
const int LeftMotor = 0; //// GPIO pin for the LeftMotor
const int RightMotor = 6; //// GPIO pin for the LeftMotor

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

const char* ssid = "TP-Link_E0C8";  //620@The_axis_apartments //TP-Link_E0C8
const char* password = "52665134";   //bdkU5RCVQGQP  //52665134

char choice;
char turnDirection;  // Gets 'l', 'r' or 'f' depending on which direction is obstacle free

int distanceCounter = 0;               
int numcycles = 0;  // Number of cycles used to rotate with head during moving
int roam = 0;       // Switching between automatic and manual mode of moving
int police = 0;   // Determine if police mode is start or not
 
// limits for obstacles:
const int distanceLimit = 27;           // Front distance limit in cm
const int sideDistanceLimit = 12;       // Side distance limit in cm
const int turnTime = 800;// Time needed to turn robot

// handle vive
#define RGBLED 2 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 18 // pin receiving signal from Vive circuit
#define SIGNALPIN2 19 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
#define STUDENTIP 130 // choose a teammembers assigned IP number
#define teamNumber 1
#define FREQ 1 // in Hz

Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);    

static float x3,y3,x2,y2;
int team;
float x,y;

WiFiUDP UDPServer;
WiFiUDP UDPTestServer;
IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast

void UdpSend(int x_udp, int y_udp)
{
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d:%4d,%4d",teamNumber,x_udp,y_udp);   
                                              
  UDPTestServer.beginPacket(ipTarget, UDPPORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  Serial.println(udpBuffer);
}

void trackPolice(){
  float s1 = (y-y3)/(x-x3);
  float s2 = (y3-y2)/(x3-x2);
  Serial.print("s1: ");
  Serial.println(s1);
  Serial.print("s2: ");
  Serial.println(s2);
  if (abs(s1-s2)>=0.2){
    moveRight();
    delay(200);
    moveStop();
    delay(400);
  }else{
    moveForward();
    delay(800);
  }
}

void handleUDPServer() {
   const int UDP_PACKET_SIZE = 14; // can be up to 65535          
   uint8_t packetBuffer[UDP_PACKET_SIZE];

   int cb = UDPServer.parsePacket(); // if there is no message cb=0
   if (cb) {
      packetBuffer[13]=0; // null terminate string

      UDPServer.read(packetBuffer, UDP_PACKET_SIZE);
      team = atoi((char *)packetBuffer+0); //##,####,#### 0nd indexed char
      if(team == 0){
        x = atoi((char *)packetBuffer+3); // ##,####,#### 2nd indexed char
        y = atoi((char *)packetBuffer+8); // ##,####,#### 7th indexed char
        Serial.print("From Team ");
        Serial.println((char *)packetBuffer);
      }
   }
}

void setup(){
  int i=0;
  Serial.begin(9600);  
  
  ledcSetup(Motor_channel1, Motor_freq, Motor_resolution_bits); // setup Head Motor channel1
  ledcAttachPin(HeadPin, Motor_channel1);
  ledcSetup(Motor_channel2, Motor_freq, Motor_resolution_bits); // setup Left Motor channel2
  ledcAttachPin(LeftMotor, Motor_channel2);
  ledcSetup(Motor_channel3, Motor_freq, Motor_resolution_bits); // setup Left Motor channel2
  ledcAttachPin(RightMotor, Motor_channel3);

  // ultrasonic pin set
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  IPAddress myIP(192,168,1,130);  //change to your own IP address
  IPAddress routerIP(192,168,1,1); //192,168,1,1 //10,20,104,1
  
  WiFi.mode(WIFI_AP_STA);    //AP-static mode
  WiFi.begin(ssid, password);
  WiFi.config(myIP,routerIP,IPAddress(255,255,255,0));

  Serial.printf("team  #%d ", teamNumber); 
  while(WiFi.status()!= WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  } //check WiFi conncetion
  Serial.printf("connected to %s on",ssid); 
  Serial.println(myIP);

  UDPTestServer.begin(UDPPORT);
  UDPServer.begin(UDPPORT);
  
  vive1.begin();
  vive2.begin();
  Serial.println("  Vive trackers started");

  h.begin();   //start the server
  h.attachHandler("/",handleRoot);
  h.attachHandler("/setDirection?val=", handleSlider1);    //attach sliders
  h.attachHandler("/setMode?val=", handleSlider2);
  h.attachHandler("/setLR?val=", handleSlider3);
  
  moveStop();
}

void handleRoot(){
  h.sendhtml(body);
}

void handleSlider1() {
  int sliderValue = h.getVal();
  String s = "Move";
  if (sliderValue == 3) {
    s = s+ " Backward";
    moveBackward();
    Serial.println(sliderValue);
  } else if(sliderValue == 1) {
    s = s+ " Forward";
    moveForward();
    Serial.println(sliderValue);
  } else if(sliderValue == 2) {
    s = s+ " Stop";
    moveStop();
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
    roam = 1; 
    police = 0;
    s = s+ "Automatic";
  } else if(sliderValue == 0) {
    Serial.println("Manual mode");
    roam = 0;
    police = 0;
    moveStop();
    s = s+ "Manual";
    Serial.println(sliderValue);
  }else if(sliderValue == 2) {
    // uses existing vive value to track police car
    Serial.println("Trophy mode");
    roam = 0;
    police = 0;
    moveStop();
    s = s+ "Trophy";
    Serial.println(sliderValue);
  }else if(sliderValue == 3) {
    Serial.println("Fake mode");
    roam = 0;
    police = 0;
    moveStop();
    s = s+ "Fake";
    Serial.println(sliderValue);
  } else if(sliderValue == 4) {
    Serial.println("Police mode");
    roam = 0;
    police = 1;
    s = s+ "Police car";
    Serial.println(sliderValue);
  } 
  h.sendplain(s);
} // Mode slider: control/autonomous

void handleSlider3() {
  int sliderValue = h.getVal();
  String s = "Move";
  if (sliderValue == 3) {
    s = s+ " Right";
    moveRight();
    delay(turnTime);
    moveStop();
    Serial.println(sliderValue);
  } else if(sliderValue == 1) {
    s = s+ " Left";
    moveLeft();
    delay(turnTime);
    moveStop();
    Serial.println(sliderValue);
  } else if(sliderValue == 2) {
    s = s+ " Stop";
    moveStop();
    Serial.println(sliderValue);
  }
  h.sendplain(s);
} // mode slider: move forward, stop, or backward

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

//This function tells the robot to go forward 
void moveForward(){
  Serial.println("");
  Serial.println("Moving forward");
  ledcAnalogWrite(Motor_channel2,map(180, 0, 180, 122, 492));
  ledcAnalogWrite(Motor_channel3,map(62, 0, 180, 122, 492));
}

//This function tells the robot to move backward
void moveBackward(){
  Serial.println("");
  Serial.println("Moving backward");
  ledcAnalogWrite(Motor_channel2,map(63, 0, 180, 122, 492));
  ledcAnalogWrite(Motor_channel3,map(180, 0, 180, 122, 492));
}

//This function tells the robot to turn left
void moveRight(){
  Serial.println("");
  Serial.println("Moving right");
  ledcAnalogWrite(Motor_channel2,map(120, 0, 180, 122, 492));
  ledcAnalogWrite(Motor_channel3,map(120, 0, 180, 122, 492));
}

//This function tells the robot to turn right
void moveLeft(){
  Serial.println("");
  Serial.println("Moving left");
  ledcAnalogWrite(Motor_channel2,map(60, 0, 180, 122, 492));
  ledcAnalogWrite(Motor_channel3,map(60, 0, 180, 122, 492));
}

//This function tells the robot to stop moving
void moveStop(){
  Serial.println("");
  Serial.println("Stopping");
  ledcAnalogWrite(Motor_channel2,map(87, 0, 180, 122, 492));
  ledcAnalogWrite(Motor_channel3,map(88, 0, 180, 122, 492));
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

  // Scanning right diagnal
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
        moveStop();
        break;
      case 'r':
        moveRight();
        delay(turnTime);
        moveStop();
        break;
      case 'b':
        moveBackward();
        delay(2*turnTime);
        moveStop();
        break;
      case 'f':
        break;
    }
    distanceCounter = 0;
  }
}

void loop(){
  h.serve();
  handleUDPServer();
  static long int ms = millis();

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
    UdpSend(x3,y3);
    UdpSend(x2,y2);
  }
  
  if (vive1.status() == VIVE_RECEIVING) {
    x3 = vive1.xCoord();
    y3 = vive1.yCoord();
    neopixelWrite(RGBLED,0,x3/200,y3/200);  // blue to greenish
  }
  else {
    x3=0;
    y3=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }

  if (vive2.status() == VIVE_RECEIVING) {
    x2 = vive2.xCoord();
    y2 = vive2.yCoord();
    //neopixelWrite(RGBLED,0,x/200,y/200);  // blue to greenish
  }
  else {
    x2=0;
    y2=0; 
    switch (vive2.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }
  if(police == 1){
    trackPolice();
  }
  
  if(roam == 1){
    go();
  }
  delay(20);
}
