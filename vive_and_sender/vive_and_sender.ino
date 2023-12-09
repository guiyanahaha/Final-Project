/**
 * Simple sender  to only a receiver ESPNOW with known MAC hard coded to MAC_RECV
*/

#include <esp_now.h>
#include <WiFi.h>
#include "vive510.h"

#include <WiFiUdp.h>

#define RGBLED 2 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 5 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
#define STUDENTIP 130 // choose a teammembers assigned IP number
#define teamNumber 1
#define FREQ 1 // in Hz

#define CHANNEL 1                  // channel can be 1 to 14, channel 0 means current channel.  
#define MAC_RECV  {0x60, 0x55, 0xF9, 0x57, 0x3F, 0xA8} // receiver MAC address (last digit should be even for STA)


const char* ssid     = "TP-Link_E0C8";
const char* password = "52665134";

Vive510 vive1(SIGNALPIN1);
WiFiUDP UDPTestServer;
IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast

void UdpSend(int x, int y)
{
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d:%4d,%4d",teamNumber,x,y);   
                                              
  UDPTestServer.beginPacket(ipTarget, UDPPORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  Serial.println(udpBuffer);
}

esp_now_peer_info_t peer1 = 
{
  .peer_addr = MAC_RECV, 
  .channel = CHANNEL,
  .encrypt = false,
};

// callback when data is sent 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println ("Success ");
  else Serial.println("Fail "); 
}

void setup() {
  int i=0;
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);  

  // UDP 
  WiFi.config(IPAddress(192, 168, 1, STUDENTIP), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  Serial.printf("team  #%d ", teamNumber); 
  Serial.print("Connecting to ");  Serial.println(ssid);
  while(WiFi.status()!=WL_CONNECTED && i++ < 20){
    delay(500);   Serial.print(".");
  }
  if (i<19) {
    Serial.println("WiFi connected as "); Serial.print(WiFi.localIP());
  } else {
    Serial.printf("Could not connect err: %d ",i); 
  }
  
  UDPTestServer.begin(UDPPORT);
  
  vive1.begin();
  Serial.println("  Vive trackers started");

  // ESP NOW
  Serial.print("Sending MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("init failed");
    ESP.restart();
  }
  
  esp_now_register_send_cb(OnDataSent); //optional if you want ack interrupt
    
  if (esp_now_add_peer(&peer1) != ESP_OK) {
    Serial.println("Pair failed");     // ERROR  should not happen
  }
}

void loop() {
  // UDP
    static long int ms = millis();
  static uint16_t x,y;

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
    UdpSend(x,y);
  }
  
  if (vive1.status() == VIVE_RECEIVING) {
    x = vive1.xCoord();
    y = vive1.yCoord();
    neopixelWrite(RGBLED,0,x/200,y/200);  // blue to greenish
  }
  else {
    x=0;
    y=0; 
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
    
  delay(20);

  // ESP NOW
  static int count;
  uint8_t message[200]; // Max ESPnow packet is 250 byte data
  
  // put some message together to send

  sprintf((char *) message, "sender %d ", count++);
  
  if (esp_now_send(peer1.peer_addr, message, sizeof(message))==ESP_OK) 
    Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n",message, peer1.peer_addr[0],peer1.peer_addr[1],peer1.peer_addr[2],peer1.peer_addr[3],peer1.peer_addr[4],peer1.peer_addr[5]);   
  else Serial.println("Send failed");
  
  delay(500); // ESPNow max sending rate (with default speeds) is about 50Hz
   
}

