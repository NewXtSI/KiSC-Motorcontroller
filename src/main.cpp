#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include "../KiSC-hoverboard-protocol/include/kisc-hoverboard-protocol.h"

void recCallback(kisc::protocol::espnow::KiSCMessage message) {
    Serial.println("Received message");
    if (message.command == kisc::protocol::espnow::Command::MotorControl) {
        Serial.println("Motor control message");
    }
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    onKiSCMessageReceived(recCallback);
    initESPNow();
    Serial.printf("\n\n---- Motor Controller ----\n");
    Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());
}

void sendFeedback() {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::MotorFeedback;
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
}

void sendHeartbeat() {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::Ping;
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
}

uint32_t lastHeartbeat = 0;

void loop() {
    loopESPNow();
    sendFeedback();
    delay(10);
    if (millis() - lastHeartbeat > 2000) {
        Serial.printf("Sending heartbeat\n");
        sendHeartbeat();
        lastHeartbeat = millis();
    }
}