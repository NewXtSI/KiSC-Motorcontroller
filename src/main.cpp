#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include "../KiSC-hoverboard-protocol/include/kisc-hoverboard-protocol.h"
#include <SoftwareSerial.h>

#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define HOVER_SERIAL_RX     2           // [-] RX pin for HoverSerial
#define HOVER_SERIAL_TX     3           // [-] TX pin for HoverSerial

uint32_t        lastReceivedMessage = millis();
bool            bControllerConnected  = false;
bool            bMotorConnected       = false;
bool            inShutdown = false;
bool            bCommunicationEnabled = true;

SerialCommand  command;
SerialFeedback feedback;
SerialFeedback newFeedback;

SoftwareSerial HoverSerial(HOVER_SERIAL_RX,HOVER_SERIAL_TX);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;


void recCallback(kisc::protocol::espnow::KiSCMessage message) {
    Serial.println("Received message");
    if (message.command == kisc::protocol::espnow::Command::MotorControl) {
        lastReceivedMessage = millis();
        bControllerConnected = true;
        inShutdown = false;
        bCommunicationEnabled = true;
        Serial.println("Motor control message");
        if (bMotorConnected) {
            command.start = VALID_HEADER;
            command.buzzer.freq = 0;
            command.buzzer.pattern = 0;
            command.led = false;
            command.poweroff = message.motorControl.poweroff;

            command.standstillAcv = message.motorControl.left.standStillOnHold;
            command.cruiseCtrlAcv = message.motorControl.left.cruiseCtrlEna;
            command.electricBrakeAmount = message.motorControl.left.electricBrakeFactor;


            command.left.enable = message.motorControl.left.enable;
            command.left.pwm = message.motorControl.left.pwm;
            command.left.ctrlTyp = (ControlType)message.motorControl.left.type;
            command.left.ctrlMod = (ControlMode)message.motorControl.left.mode;
            command.left.iMotMax = message.motorControl.left.iMotMax;
            command.left.iDcMax = 10;
            command.left.nMotMax = message.motorControl.left.nMotMax;
            command.left.fieldWeakMax = message.motorControl.left.fieldWeakMax;
            command.left.phaseAdvMax = 0;
            command.left.cruiseCtrlEna = message.motorControl.left.cruiseCtrlEna;
            command.left.nCruiseMotTgt = message.motorControl.left.cruiseMotTgt;
            command.right.enable = message.motorControl.right.enable;
            command.right.pwm = message.motorControl.right.pwm;
            command.right.ctrlTyp = (ControlType)message.motorControl.right.type;
            command.right.ctrlMod = (ControlMode)message.motorControl.right.mode;
            command.right.iMotMax = message.motorControl.right.iMotMax;
            command.right.iDcMax = 10;
            command.right.nMotMax = message.motorControl.right.nMotMax;
            command.right.fieldWeakMax = message.motorControl.right.fieldWeakMax;
            command.right.phaseAdvMax = 0;
            command.right.cruiseCtrlEna = message.motorControl.right.cruiseCtrlEna;
            command.right.nCruiseMotTgt = message.motorControl.right.cruiseMotTgt;
            command.checksum = calculateCommandChecksum(command);
            if (bCommunicationEnabled)
                HoverSerial.write((byte *)&command, sizeof(SerialCommand));
        }
    }
}

void ReceiveHoverboard() {
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

    if (bufStartFrame == VALID_HEADER) {  // Initialize if new data is detected
        p       = (byte *)&newFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte;
        idx++;
    }
        // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        // Check if the checksum is correct
        if (newFeedback.checksum == calculateFeedbackChecksum(newFeedback)) {
            // Copy the new data to the Feedback struct
            memcpy(&feedback, &newFeedback, sizeof(SerialFeedback));
        }
        idx = 0;
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

void sendFeedback() {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::MotorFeedback;
    message.motorFeedback.batVoltage = feedback.batVoltage;
    message.motorFeedback.boardTemp = feedback.boardTemp;
    message.motorFeedback.motorboardConnected = bMotorConnected;

    message.motorFeedback.left.speed = feedback.left.speed;
    message.motorFeedback.left.angle = feedback.left.angle;
    message.motorFeedback.left.error = feedback.left.error;
    message.motorFeedback.left.dcLink = feedback.left.dcLink;
    message.motorFeedback.left.dcPhaA = feedback.left.dcPhaA;
    message.motorFeedback.left.dcPhaB = feedback.left.dcPhaB;
    message.motorFeedback.left.dcPhaC = feedback.left.dcPhaC;
    message.motorFeedback.left.chops = feedback.left.chops;
    message.motorFeedback.left.id = feedback.left.id;
    message.motorFeedback.left.iq = feedback.left.iq;
    message.motorFeedback.left.hallA = feedback.left.hallA;
    message.motorFeedback.left.hallB = feedback.left.hallB;
    message.motorFeedback.left.hallC = feedback.left.hallC;
    message.motorFeedback.right.speed = feedback.right.speed;
    message.motorFeedback.right.angle = feedback.right.angle;
    message.motorFeedback.right.error = feedback.right.error;
    message.motorFeedback.right.dcLink = feedback.right.dcLink;
    message.motorFeedback.right.dcPhaA = feedback.right.dcPhaA;
    message.motorFeedback.right.dcPhaB = feedback.right.dcPhaB;
    message.motorFeedback.right.dcPhaC = feedback.right.dcPhaC;
    message.motorFeedback.right.chops = feedback.right.chops;
    message.motorFeedback.right.id = feedback.right.id;
    message.motorFeedback.right.iq = feedback.right.iq;
    message.motorFeedback.right.hallA = feedback.right.hallA;
    message.motorFeedback.right.hallB = feedback.right.hallB;
    message.motorFeedback.right.hallC = feedback.right.hallC;
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
}

void sendHeartbeat() {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::Ping;
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
}

uint32_t lastHeartbeat = millis();

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    HoverSerial.begin(HOVER_SERIAL_BAUD);
    onKiSCMessageReceived(recCallback);
    initESPNow();
    Serial.printf("\n\n---- Motor Controller ----\n");
    Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());
}

void ShutdownMotors() {
    inShutdown = true;
}

void loop() {
    loopESPNow();
//    sendFeedback();
    if (millis() - lastReceivedMessage > 1000) {
        if (bControllerConnected) {
            Serial.printf("Controller not connected\n");
            bControllerConnected = false;
            ShutdownMotors();
        }
    }
    delay(10);
    if (millis() - lastHeartbeat > 2000) {
        Serial.printf("Sending heartbeat\n");
        sendHeartbeat();
        lastHeartbeat = millis();
        if (!bMotorConnected) {
            Serial.printf("Motor not connected\n");
            sendFeedback();
        }
    }
    if (bMotorConnected) {
        if (inShutdown) {
            if (bMotorConnected) {
                command.start = VALID_HEADER;
                command.buzzer.freq = 0;
                command.buzzer.pattern = 0;
                command.led = false;
                command.poweroff = true;
                command.left.enable = false;
                command.right.enable = false;
                command.checksum = calculateCommandChecksum(command);
                HoverSerial.write((byte *)&command, sizeof(SerialCommand));
                bCommunicationEnabled = false;
            }
        }
        sendFeedback();
    }
    // Check for new received data
    ReceiveHoverboard();
}