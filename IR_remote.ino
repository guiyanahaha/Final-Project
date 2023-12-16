#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>

#include "driver/i2c.h"
#include "sdkconfig.h"
#include "vive510.h"
#include "Webpage.h"
#include "html510.h"

const float a = 0.239; // Updated filter coefficient for 50 Hz cutoff
float filteredValue = 0; // Holds the filtered value
int lastReading = 0; // Holds the last reading
unsigned long lastCrossingTime = 0; // Time of the last zero crossing
bool crossingDetected = false; // Flag to avoid multiple detections for the same crossing

// handle master-slave(i2c)
#define DATA_LENGTH 128                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 22               /*!< Data length for r/w test, [0,DATA_LENGTH] */

#define I2C_SLAVE_SCL_IO (gpio_num_t)7               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO (gpio_num_t)10              /*!< gpio number for i2c slave data */
#define I2C_SLAVE_TX_BUF_LEN (2* DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define ESP_SLAVE_ADDR       0x28 /*!< ESP32 slave address, you can set any 7bit value */

#define IR_PIN     0
#define TRIG_PIN     4                // GPIO pin for triger on ultrasonic sensor
#define ECHO_PIN     5              // GPIO pin for echo 
#define Motor_channel1 0 
#define Motor_channel2 1 
#define Motor_channel3 2

const int HeadPin = 1;  // GPIO pin for the Head Motor
const int LeftMotor = 8; //// GPIO pin for the LeftMotor
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
char lenStr[20];

const char* ssid = "TP-Link_E0C8";  //620@The_axis_apartments //TP-Link_E0C8
const char* password = "52665134";   //bdkU5RCVQGQP  //52665134

char choice;
char turnDirection;  // Gets 'l', 'r' or 'f' depending on which direction is obstacle free

int distanceCounter = 0;               
int numcycles = 0;  // Number of cycles used to rotate with head during moving
int roam = 0;       // Switching between automatic and manual mode of moving
int police = 0;   // Determine if police mode is start or not
 
// limits for obstacles:
const int distanceLimit = 35;           // Front distance limit in cm
const int sideDistanceLimit = 25;       // Side distance limit in cm
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
int team, count;
float x,y;
float freq;  

int x3_buffer[10], y3_buffer[10],x2_buffer[10], y2_buffer[10];

WiFiUDP UDPServer;
WiFiUDP UDPTestServer;
IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast

int typo, fake;
String message = "b";
uint8_t data_rd[DATA_LENGTH];
uint8_t data_wr[] = "Team 1";

/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init()
{
    i2c_port_t i2c_slave_port = I2C_NUM_0;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    return i2c_driver_install(i2c_slave_port, conf_slave.mode,
                              I2C_SLAVE_RX_BUF_LEN,
                              I2C_SLAVE_TX_BUF_LEN, 0);
}

void UdpSend(int x_udp, int y_udp)
{
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d:%4d,%4d",teamNumber,x_udp,y_udp);   
                                              
  UDPTestServer.beginPacket(ipTarget, UDPPORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  Serial.println(udpBuffer);
}

void trackfake(){
  if (abs(freq-23)<=2){
    moveForward();
  } else if(abs(freq-23)>2){
    moveLeft();
  }
}

void trackPolice(){
int deltax = x-findmedian(x3_buffer,10);
  int deltay = y-findmedian(y3_buffer,10);
  int deltaRX = findmedian(x3_buffer,10)-findmedian(x2_buffer,10);
  int deltaRY = findmedian(y3_buffer,10)-findmedian(y2_buffer,10);
  float angle = atan2(deltay, deltax);
  float robot = atan2(deltaRY, deltaRX);
  float robotx = (angle - robot)*180/3.14;
  Serial.print("robotx:");
  Serial.println(robotx);
  float mag = sqrt(sq(deltax)+sq(deltay));
    if (abs(robotx)>=20 && mag >= 500){
      moveRight();
      delay(200);
      moveStop();
      delay(600);
    }else if(abs(robotx)<20 && mag > 500){
      moveForward();
      delay(3000);
    } else if (mag <500) {
      moveBackward();
      delay(500);
      moveForward();
      delay(1000);
    } else {
      moveStop();
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

  i2c_slave_init();

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
  message = s;
  int len = message.length();
  strcpy((char*)data_wr, "Team 1: "); // Copy "Team 1: " to data_wr
  strcat((char*)data_wr, itoa(len*8, lenStr, 10)); // Concatenate message to data_wr
  typo = 1;
  
  h.sendplain(s);
} // mode slider: move forward, stop, or backward

void handleSlider2() {
  int sliderValue = h.getVal();
  String s = "";
  if (sliderValue == 1){
    Serial.println("Automatic mode");
    moveStop();
    roam = 1; 
    fake = 0;
    police = 0;
    s = s+ "Automatic";
  } else if(sliderValue == 0) {
    Serial.println("Manual mode");
    roam = 0;
    fake = 0;
    police = 0;
    moveStop();
    s = s+ "Manual control";
    Serial.println(sliderValue);
  }else if(sliderValue == 2) {
    // uses existing vive value to track police car
    Serial.println("Trophy mode");
    roam = 0;
    fake = 0;
    police = 0;
    moveStop();
    s = s+ "Trophy";
    Serial.println(sliderValue);
  }else if(sliderValue == 3) {
    Serial.println("Fake mode");
    roam = 0;
    fake = 1;
    police = 0;
    moveStop();
    s = s+ "Fake";
    Serial.println(sliderValue);
  } else if(sliderValue == 4) {
    Serial.println("Police mode");
    roam = 0;
    fake = 0;
    police = 1;
    s = s+ "Police car";
    Serial.println(sliderValue);
  } 
  message = s;
  int len = message.length();
  strcpy((char*)data_wr, "Team 1: "); // Copy "Team 1: " to data_wr
  strcat((char*)data_wr, itoa(len*8, lenStr, 10)); // Concatenate message to data_wr
  typo = 1;
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
  message = s;
  int len = message.length();
  strcpy((char*)data_wr, "Team 1: "); // Copy "Team 1: " to data_wr
  strcat((char*)data_wr, itoa(len*8, lenStr, 10)); // Concatenate message to data_wr
  typo = 1;
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
  ledcAnalogWrite(Motor_channel2,map(110, 0, 180, 122, 492));
  ledcAnalogWrite(Motor_channel3,map(0, 0, 180, 122, 492));
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
  ledcAnalogWrite(Motor_channel2,map(88, 0, 180, 122, 492));
  ledcAnalogWrite(Motor_channel3,map(87, 0, 180, 122, 492));
}


void watchsurrounding(){

  // Scanning front
  ledcAnalogWrite(Motor_channel1,map(130, 0, 180, 122, 492));  // update Motor duty cycle
  delay(300);
  scan();
  FrontDistance = distance;

  // Scanning right
  ledcAnalogWrite(Motor_channel1,map(50, 0, 180, 122, 492));  // update Motor duty cycle
  delay(300);
  scan();
  RightDistance = distance;
}


void go() {
// wall following main code
  if (FrontDistance >= distanceLimit && -5 < (RightDistance - sideDistanceLimit) && (RightDistance - sideDistanceLimit) < 5) {
    moveForward();
    
  }
  else if (FrontDistance < distanceLimit) {
    moveLeft();
    delay(600);
    moveStop();
    moveForward();
  }
  if (FrontDistance >= distanceLimit && RightDistance <= (sideDistanceLimit - 5)) {
    moveLeft();
    delay(100);
    moveStop();
    moveForward();
  }
  else if (FrontDistance >= distanceLimit && RightDistance >= (sideDistanceLimit + 5)) {
    moveRight();
    delay(100);
    moveStop();
    moveForward();
  }
  // else if (-5 < (RightDistance - sideDistanceLimit) && (RightDistance - sideDistanceLimit) < 5) {
  //   moveForward();
  // }
}

int findmedian(int arr[], int size) {
  // Sort the array
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (arr[j] < arr[i]) {
        // Swap elements if they are in the wrong order
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }

  // Calculate the median
  if (size % 2 == 0) {
    // If the size of the array is even, return the average of the middle two elements
    return (arr[size / 2 - 1] + arr[size / 2]) / 2;
  } else {
    // If the size of the array is odd, return the middle element
    return arr[size / 2];
  }
}

void loop(){
  h.serve();
//  // Read the analog value (example)
//  int sensorValue = analogRead(0); 
//
//  // Convert to a voltage, assuming a 3.3V system and 12-bit ADC resolution
//  float voltage = sensorValue * (3.3 / 4095.0);
//
//  // Apply the low-pass filter with the new coefficient
//  filteredValue = a * voltage + (1 - a) * filteredValue;
//
//  // Check for zero crossing
//  if ((lastReading <= 0 && filteredValue > 0) || (lastReading >= 0 && filteredValue < 0)) {
//    if (!crossingDetected) {
//      unsigned long currentTime = millis();
//      if (lastCrossingTime > 0) { // Skip the first crossing
//        unsigned long period = currentTime - lastCrossingTime;
//        if (period > 0) { // Avoid division by zero
//          float frequency = 1000.0 / period; // Calculate frequency
//          freq = frequency;
//          Serial.print("Frequency: ");
//          Serial.println(freq);
//        }
//      }
//      lastCrossingTime = currentTime;
//      crossingDetected = true;
//    }
//  } else {
//    crossingDetected = false;
//  }
//
//  lastReading = filteredValue;
  
    handleUDPServer();
    static long int ms = millis();
    
 //master-slave communication
    if (i2c_slave_read_buffer(I2C_NUM_0, data_rd, RW_TEST_LENGTH, 0) > 0 ) { // last term is timeout period, 0 means don't wait  
        Serial.printf("READ from master: %s\n",data_rd);
      //I2Cport buffer length of data     max ticks to wait if buffer is full
      if (i2c_slave_write_buffer(I2C_NUM_0, data_wr, RW_TEST_LENGTH, 10 / portTICK_RATE_MS) ) {
        Serial.printf("WRITE to master: %s\n",data_wr);
      }  
    }

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
    x3_buffer[count] = x3;
    y3_buffer[count] = y3;
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
  
  x3_buffer[count] = x3;
  y3_buffer[count] = y3;
  x2_buffer[count] = x2;
  y2_buffer[count] = y2;

  if(police == 1){
    count++;
  }
  
  if(police == 1 && count>= 10){
    trackPolice();
    count = 0;
  }
  
  if(roam == 1){
    watchsurrounding();
    go();
  }

  if(fake == 1){
    //trackfake();
  }
  delay(1);
}
