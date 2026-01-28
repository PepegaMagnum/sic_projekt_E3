#include <LoRa.h>
#include "CRC16.h"

#define ID_S1 0
#define ID_W1 1
#define ID_W2 2
#define ID_W3 3
#define ID_W4 4
#define ID_W5 5
#define ID_W0 6

const uint8_t MY_NODE_ID = ID_W0;

#define S0 4
#define S1 5
#define SENSOR_OUT 6
#define S2 7
#define S3 8

CRC16 crc16;
const unsigned long DELAY = 500; // ms
const unsigned long TRANSMISSION_DELAY = 70; // ms
const uint8_t       CNT_RESET_VALUE = (1000/DELAY)*10;
const bool          DEBUG = false;

unsigned long lastSeen [6];
unsigned long now;
bool active[6];

typedef struct DataFrame {
  uint8_t messageId;
  uint8_t senderId;
  uint8_t path;
  uint16_t data;
} DataFrame_t;

typedef struct Packet {
  DataFrame_t payload;
  uint16_t crc;
} Packet_t;

Packet_t packet;
Packet_t packet_copy;

uint16_t rcvCrc;
uint8_t buffer[7];

void setBit(uint8_t* value, uint8_t bitIndex){
  *value |= (1<<bitIndex);
}

void clearBit(uint8_t* value, uint8_t bitIndex){
  *value &= ~(1<<bitIndex);
}

uint16_t calcCRC16(DataFrame_t payload){
  uint16_t tmp;

  crc16.add(payload.messageId);
  crc16.add(payload.senderId);
  crc16.add(payload.path);
  crc16.add(highByte(payload.data));
  crc16.add(lowByte(payload.data));

  tmp = crc16.calc();
  crc16.restart();
  return tmp;
}

void setup() {
  Serial.begin(9600);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed");
    while (1);
  }

  packet.payload = {
    0, 0, 0, 0
  };

  packet.crc = 0;

  Serial.println("LoRa node ready");

}

void loop() {
  int packetSize = LoRa.parsePacket();
  // Serial.println("PacketSize:");
  // Serial.println(packetSize);
  // Serial.println("Waiting for packet...");
  if(packetSize != 0){
    Serial.println("Packet  incoming!");
    for (int i = 0; i < 7; i++){
      buffer[6] = buffer[5];
      buffer[5] = buffer[4];
      buffer[4] = buffer[3];
      buffer[3] = buffer[2];
      buffer[2] = buffer[1];
      buffer[1] = buffer[0];
      buffer[0] = LoRa.read();
    }

    // for (int i = 0; i < 7; i++){
    //   Serial.println(buffer[i], BIN);
    // }

    packet.payload.messageId = buffer[6];
    packet.payload.senderId = buffer[5];
    packet.payload.path = buffer[4];
    packet.payload.data = (buffer[3] << 8) | buffer[2];
    packet.crc = (buffer[1] << 8) | buffer[0];

    rcvCrc = calcCRC16(packet.payload);

    if (rcvCrc == packet.crc){

      Serial.println("CRC Correct! Packet received successfully");
      
      
      if (bitRead(packet.payload.path, MY_NODE_ID) == 1){
        Serial.println("This message has been here, dropping packet");
      }
      else if ((packet.payload.messageId = packet_copy.payload.messageId) and (packet.payload.senderId == packet_copy.payload.senderId)){
        Serial.println("Duplicate message");
      } else {
        Serial.println("Payload: ");
        Serial.print("MsgId: ");
        Serial.println(packet.payload.messageId);
        Serial.print("SenderId: ");
        Serial.println(packet.payload.senderId);
        Serial.print("Path: ");
        Serial.println(packet.payload.path, BIN);
        Serial.print("Data: ");
        Serial.println(packet.payload.data);
        Serial.print("CRC: ");
        Serial.println(packet.crc, BIN);

        for (int i=0; i < 5; i++){
          if (bitRead(packet.payload.path, i) == 1) {
            lastSeen[i] = millis();
          }
        }
    
        packet_copy = packet;
        memset(&packet, 0, sizeof(Packet_t));
      }
      memset(&packet, 0, sizeof(Packet_t));

    } 
    // else {
    //   Serial.println("Received CRC is not equal to calculated CRC");
    //   Serial.print("Received CRC:   "); 
    //   Serial.println(packet.crc, BIN);

    //   Serial.print("Calculated CRC: ");
    //   Serial.println(rcvCrc, BIN);
    //   idleCnt++;

    // }
    // memset(&buffer, 0, sizeof(buffer));
  } 
  now = millis();
  for (int i; i<5; i++){
    if((now-lastSeen[i]) > 10000){
      active[i] = false;
    } else {
      active[i] = true;
    }
  }
  Serial.println("Network Status: ");
  Serial.print("S0: ");
  if (active[0] == 1){
    Serial.println("active");
  } else {
    Serial.println("not active");
  }

  for (int i = 1; i < 6; i++){
    Serial.print("W");
    Serial.print(i);
    Serial.print(": ");
    if (active[i] == 1){
      Serial.println("Active");
    } else {
      Serial.println("Not Active");
    }
  }

  memset(&packet, 0, sizeof(Packet_t));
  delay(TRANSMISSION_DELAY);
}
