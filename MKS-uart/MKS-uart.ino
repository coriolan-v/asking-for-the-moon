#include "bluefruit.h"

#define SERVO_ADDR 0xE1
#define CMD_READ_ANGLE 0x30
#define LEN_READ_ANGLE 0x10
#define RESPONSE_LEN 6

uint8_t responseBuffer[RESPONSE_LEN];
uint8_t responseIndex = 0;
bool requestSent = false;
bool responseReady = false;

unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);       // Debug over USB
  Serial1.begin(57600);       // Hardware UART to MKS SERVO42C
}

void loop() {
  // Send the command once
  if (!requestSent) {
    sendReadAngleCommand();
    requestSent = true;
    lastSendTime = millis();
  }

  // Collect incoming bytes if available
  while (Serial1.available()) {
    if (responseIndex < RESPONSE_LEN) {
      responseBuffer[responseIndex++] = Serial1.read();

      // When enough bytes received, mark as ready
      if (responseIndex == RESPONSE_LEN) {
        responseReady = true;
      }
    } else {
      // Overflow, discard extra
      Serial1.read();
    }
  }

  // Process if a full response has arrived
  if (responseReady) {
    handleResponse();
    // Reset for next loop
    responseIndex = 0;
    responseReady = false;
    requestSent = false;
  }

  // Optional timeout after 200ms
  if (requestSent && millis() - lastSendTime > 200) {
    Serial.println("Timeout: No response received.");
    requestSent = false;
    responseIndex = 0;
  }
}

void sendReadAngleCommand() {
  uint8_t packet[4];
  packet[0] = SERVO_ADDR;
  packet[1] = CMD_READ_ANGLE;
  packet[2] = LEN_READ_ANGLE;
  packet[3] = (packet[0] + packet[1] + packet[2]) & 0xFF;

  Serial1.write(packet, sizeof(packet));
  Serial.println("Sent read angle command");
}

void handleResponse() {
  // Check checksum
  uint8_t checksum = 0;
  for (int i = 0; i < RESPONSE_LEN - 1; i++) {
    checksum += responseBuffer[i];
  }
  checksum &= 0xFF;

  if (checksum != responseBuffer[RESPONSE_LEN - 1]) {
    Serial.println("Invalid checksum");
    return;
  }

  // Parse raw angle
  uint16_t rawAngle = (responseBuffer[3] << 8) | responseBuffer[4];
  float angleDeg = rawAngle * 360.0 / 16384.0;

  Serial.print("Received Angle: ");
  Serial.print(angleDeg, 2);
  Serial.println("°");
}

