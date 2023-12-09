/**
 * Simple sender  to only a receiver ESPNOW with known MAC hard coded to MAC_RECV
*/

#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1                  // channel can be 1 to 14, channel 0 means current channel.  
#define MAC_RECV  {0x68,0x67,0x25,0xB4,0xC6,0x10} // receiver MAC address (last digit should be even for STA)

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

// callback on receive
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.printf(" Recv from: %02x:%02x:%02x:%02x:%02x:%02x ",mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]); 
  Serial.print(" Data: "); 
  if (isascii(data[0])) Serial.println( (char *)data);
  else {
    for (int i=0; i<data_len; i++ ){
      Serial.printf("%x",data[i]); if (i%3==0) Serial.print(" ");
    }
  }
  static int count;
  uint8_t message[200]; // Max ESPnow packet is 250 byte data

  // put some message together to send
  sprintf((char *) message, "sender %d ", count++);
  
  if (esp_now_send(peer1.peer_addr, message, sizeof(message))==ESP_OK) 
    Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n",message, peer1.peer_addr[0],peer1.peer_addr[1],peer1.peer_addr[2],peer1.peer_addr[3],peer1.peer_addr[4],peer1.peer_addr[5]);   
  else Serial.println("Send failed");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  
// send ESP now transmit
  Serial.print("Sending MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("init failed");
    ESP.restart();
  }
  
  esp_now_register_send_cb(OnDataSent); //optional if you want ack interrupt
    
  if (esp_now_add_peer(&peer1) != ESP_OK) {
    Serial.println("Pair failed");     // ERROR  should not happen
  }


// receive ESP now transmit
  Serial.print("ESPNow Receiving MAC: ");  Serial.println(WiFi.macAddress());
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  delay(25); // ESPNow max sending rate (with default speeds) is about 50Hz
}
