#include <LoRa.h>
#include "CRC16.h"

#define ID_S1 0
#define ID_W1 1
#define ID_W2 2
#define ID_W3 3
#define ID_W4 4
#define ID_W5 5
#define ID_W0 6

const uint8_t MY_NODE_ID = ID_S1;

#define S0 4
#define S1 5
#define SENSOR_OUT 6
#define S2 7
#define S3 8

CRC16 crc16;
const unsigned long DELAY = 2000;
unsigned long transmissionStart = 0;
unsigned long transmissionEnd = 0;
unsigned long transmissionTime = 0;

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
uint8_t msgID = 0;

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
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

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
  // Prepare packet
  packet.payload.messageId = msgID;
  packet.payload.senderId = MY_NODE_ID;
  packet.payload.data = (uint8_t) random(6000, 7000);
  setBit(&packet.payload.path, MY_NODE_ID);
  packet.crc = calcCRC16(packet.payload);

  // View the packet sent
  Serial.print("msgID: ");
  Serial.println(packet.payload.messageId);

  Serial.print("senderID: ");
  Serial.println(packet.payload.senderId);

  Serial.print("data: ");
  Serial.println(packet.payload.data);

  Serial.print("path: ");
  Serial.println(packet.payload.path, BIN);

  Serial.print("CRC: ");
  Serial.println(packet.crc, BIN );

  transmissionStart = millis();
  LoRa.beginPacket();
  LoRa.write(packet.payload.messageId);
  LoRa.write(packet.payload.senderId);
  LoRa.write(packet.payload.path);
  LoRa.write(highByte(packet.payload.data));
  LoRa.write(lowByte(packet.payload.data));
  LoRa.write(highByte(packet.crc));
  LoRa.write(lowByte(packet.crc));
  LoRa.endPacket();
  Serial.println("Packet sent");
  transmissionEnd = millis();

  transmissionTime = transmissionEnd - transmissionStart;

  Serial.print("Transmission Time: ");
  Serial.println(transmissionTime);
  if (msgID = 200){
    msgID = 0;
  } else {
    msgID++;
  }
  
  delay(DELAY);
}
