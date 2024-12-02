#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#else
#include <WiFi.h>
#endif

#define ESP32DEBUGGING
#include <ESP32Logger.h>

#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include "../KiSC-hoverboard-protocol/include/kisc-hoverboard-protocol.h"
#include "../KiSC-hoverboard-protocol/include/new-protocol.h"

#define SERIAL_OUTPUT_ENABLED       1

#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define HOVER_SERIAL_BAUD_MAX 116000     // [-] Timeout for HoverSerial communication
#define HOVER_SERIAL_BAUD_INC 5
#ifdef ESP8266
#define HOVER_SERIAL_RX     D5           // [-] RX pin for HoverSerial
#define HOVER_SERIAL_TX     D6           / / [-] TX pin for HoverSerial
#else
#define HOVER_SERIAL_RX     39           // [-] RX pin for HoverSerial
#define HOVER_SERIAL_TX     40           // [-] TX pin for HoverSerial
#endif
#define DEBUG_COMM_SERIAL_RECEIVE   1
#define DEBUG_COMM_SERIAL_SEND      0

// Time after a ping is sent, when there is no other communication
#define HOVER_PING_INTERVAL 120
#define HOVER_SEND_INTERVAL 10

#if 1

#define HoverSerial Serial1

#define MAX_RECV_LEN 64
uint8_t uiReceiveBuffer[MAX_RECV_LEN+1];

typedef struct {
    int16_t rpm;
    int16_t target;
    bool enabled;
    bool error;
} HoverboardMotorStatus;

typedef enum {
    hoverBoardOffline,
    hoverBoardOnline,
    hoverBoardStarting,
    hoverBoardStopping,
    hoverBoardError,
    hoverBoardUnknown
} HoverboardSerialState;

typedef struct {
    HoverboardSerialState state;
    uint16_t voltage;
    uint16_t temperature;
    uint16_t current;
    HoverboardMotorStatus left;
    HoverboardMotorStatus right;
} HoverboardStatus;

HoverboardStatus hoverboardStatus;
HoverboardStatus hoverboardWantedStatus;

SemaphoreHandle_t hoverboardStatusMutex;
SemaphoreHandle_t hoverboardReceiveBufferMutex;

bool messageDirtyFlag = true;

kisc::protocol::espnow::KiSCMessage motorStatusMessage;
uint32_t    lastSentKiSCMessage = 0;

void hoverProtocolParse() {
    uint8_t cmd = uiReceiveBuffer[2];
    switch (cmd) {
        case HOVER_CMD_PING:
            break;
        case HOVER_CMD_STATUS:
            if (xSemaphoreTake(hoverboardStatusMutex, portMAX_DELAY) == pdTRUE) {
                hoverboardStatus.voltage = (uiReceiveBuffer[4] << 8) | uiReceiveBuffer[5];
                hoverboardStatus.temperature = (uiReceiveBuffer[6] << 8) | uiReceiveBuffer[7];
                hoverboardStatus.current = (uiReceiveBuffer[8] << 8) | uiReceiveBuffer[9];
                xSemaphoreGive(hoverboardStatusMutex);
                DBGLOG(Verbose, "Volt: %4.2f V Temp: %4.1f °C Curr: %4.2f A",
                                hoverboardStatus.voltage / 100.0, hoverboardStatus.temperature / 10.0,
                                hoverboardStatus.current / 10.0);

                if (motorStatusMessage.motorFeedback.batVoltage != hoverboardStatus.voltage) {
                    motorStatusMessage.motorFeedback.batVoltage = hoverboardStatus.voltage;
                    messageDirtyFlag = true;
                }
                if (motorStatusMessage.motorFeedback.boardTemp != hoverboardStatus.temperature) {
                    motorStatusMessage.motorFeedback.boardTemp = hoverboardStatus.temperature;
                    messageDirtyFlag = true;
                }
                if (messageDirtyFlag) {
                    sendKiSCMessage(MAIN_CONTROLLER_MAC, motorStatusMessage);
                    messageDirtyFlag = false;
                    lastSentKiSCMessage = millis();
                }
            }
            break;
        case HOVER_CMD_MOTORSTAT:
            if (xSemaphoreTake(hoverboardStatusMutex, portMAX_DELAY) == pdTRUE) {
                hoverboardStatus.left.rpm = (uiReceiveBuffer[4] << 8) | uiReceiveBuffer[5];
                hoverboardStatus.right.rpm = (uiReceiveBuffer[6] << 8) | uiReceiveBuffer[7];
                hoverboardStatus.left.target = (uiReceiveBuffer[8] << 8) | uiReceiveBuffer[9];
                hoverboardStatus.right.target = (uiReceiveBuffer[10] << 8) | uiReceiveBuffer[11];
                hoverboardStatus.left.enabled = uiReceiveBuffer[12] & 0x01;
                hoverboardStatus.left.error = uiReceiveBuffer[12] & 0x02;
//                hoverboardStatus.left.cruiseControl = uiReceiveBuffer[12] & 0x04;
                hoverboardStatus.right.enabled = uiReceiveBuffer[13] & 0x01;
                hoverboardStatus.right.error = uiReceiveBuffer[13] & 0x02;
//                hoverboardStatus.right.cruiseControl = uiReceiveBuffer[13] & 0x04;
                xSemaphoreGive(hoverboardStatusMutex);
                DBGLOG(Verbose, "Left: RPM: %4d Tgt: %4d Ena: %1d Err: %1d Right: RPM: %4d Tgt: %4d Ena: %1d Err: %1d",
                              hoverboardStatus.left.rpm, hoverboardStatus.left.target, hoverboardStatus.left.enabled, hoverboardStatus.left.error,
                              hoverboardStatus.right.rpm, hoverboardStatus.right.target, hoverboardStatus.right.enabled, hoverboardStatus.right.error);
                if (motorStatusMessage.motorFeedback.left.speed != hoverboardStatus.left.rpm) {
                    motorStatusMessage.motorFeedback.left.speed = hoverboardStatus.left.rpm;
                    messageDirtyFlag = true;
                }
                if (motorStatusMessage.motorFeedback.right.speed != hoverboardStatus.right.rpm) {
                    motorStatusMessage.motorFeedback.right.speed = hoverboardStatus.right.rpm;
                    messageDirtyFlag = true;
                }
                if (motorStatusMessage.motorFeedback.left.target != hoverboardStatus.left.target) {
                    motorStatusMessage.motorFeedback.left.target = hoverboardStatus.left.target;
                    messageDirtyFlag = true;
                }
                if (motorStatusMessage.motorFeedback.right.target != hoverboardStatus.right.target) {
                    motorStatusMessage.motorFeedback.right.target = hoverboardStatus.right.target;
                    messageDirtyFlag = true;
                }
                if (motorStatusMessage.motorFeedback.left.error != hoverboardStatus.left.error) {
                    motorStatusMessage.motorFeedback.left.error = hoverboardStatus.left.error;
                    messageDirtyFlag = true;
                }
                if (motorStatusMessage.motorFeedback.right.error != hoverboardStatus.right.error) {
                    motorStatusMessage.motorFeedback.right.error = hoverboardStatus.right.error;
                    messageDirtyFlag = true;
                }
                if (messageDirtyFlag) {
                    sendKiSCMessage(MAIN_CONTROLLER_MAC, motorStatusMessage);
                    messageDirtyFlag = false;
                    lastSentKiSCMessage = millis();
                }
            }
            break;
        default:
            DBGLOG(Warning, "Unknown command %02X", cmd);
            break;
    }
}

uint32_t lastReceivedByte = millis();
uint32_t lastReceivedMessage = millis();

void hoverReceive() {
    static uint16_t bufStartFrame;
    uint8_t incomingByte;
    static uint8_t incomingBytePrev;
    static uint8_t idx = 0;
    static byte *p;
    while (HoverSerial.available()) {
//    if (HoverSerial.available()) {
        incomingByte = HoverSerial.read();
        lastReceivedByte = millis();
        bufStartFrame = ((uint16_t)(incomingBytePrev) << 8) | incomingByte;
//    } else {
//        return;
//    }
        //  |  0 |  1 |  2  |  3  | 4... | len-1 |
        //  | A5 | A5 | CMD | LEN | DATA | CHK |
        //Serial.printf("%02X ", incomingByte);
    if ((bufStartFrame == HOVER_VALID_HEADER) && (idx == 0)) {  // Initialize if new data is detected
        p = reinterpret_cast<byte *>(&uiReceiveBuffer);
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;
    } else if (idx >= 2) {  // Save the new received data
        if (xSemaphoreTake(hoverboardReceiveBufferMutex, portMAX_DELAY) == pdTRUE) {
            *p++    = incomingByte;
            idx++;
            xSemaphoreGive(hoverboardReceiveBufferMutex);
        }
    }
    if (idx >= 4) {
        uint8_t len = uiReceiveBuffer[3];
        if (idx >= len) {
            uint8_t cmd = uiReceiveBuffer[2];
            uint8_t checksum = 0;
            for (int i = 0; i < len-1; i++) {
                checksum ^= uiReceiveBuffer[i];
            }
            if (checksum == uiReceiveBuffer[len-1]) {
                lastReceivedMessage = millis();
                hoverProtocolParse();
                if ((hoverboardStatus.state == hoverBoardError) || (hoverboardStatus.state == hoverBoardStarting)) {
                    hoverboardStatus.state = hoverBoardOnline;
                    DBGLOG(Info, "Hoverboard onlne");
                    motorStatusMessage.motorFeedback.motorboardConnected = true;
                }
            } else {
                DBGLOG(Error, "Checksum error (is: %02X calc: %02X)", uiReceiveBuffer[len-1], checksum);
            }
            idx = 0;
            bufStartFrame = 0x0000;
        }
    }
    // Check if we reached the end of the package
    if (idx >= MAX_RECV_LEN) {
        DBGLOG(Warning, "Buffer overflow");
        idx = 0;
        bufStartFrame = 0x0000;
    }
    incomingBytePrev = incomingByte;
    }
}

uint32_t lastSentToHover = millis();

void sendToHover(uint8_t cmd, uint8_t *data, uint8_t len) {
    char buf[128];
    buf[0] = (HOVER_VALID_HEADER & 0xFF00) >> 8;
    buf[1] = HOVER_VALID_HEADER & 0xFF;
    buf[2] = cmd;
    buf[3] = len+5;
    for (int i = 0; i < len; i++) {
        buf[4+i] = data[i];
    }
    uint8_t checksum = 0;
    for (int i = 0; i < len+4; i++) {
        checksum ^= buf[i];
    }
    buf[len+4] = checksum;
    HoverSerial.write((byte *)buf, len+5);
    lastSentToHover = millis();
}

// Hoverboard commands
void hoverShutdown() {
    uint8_t data[HOVER_CMD_POWER_SIZE];
    data[0] = 0xAA;
    sendToHover(HOVER_CMD_POWER, data, sizeof(data));
}

void hoverPing() {
    sendToHover(HOVER_CMD_PING, nullptr, 0);
}

void hoverBoardSendMotorCommand() {
    uint8_t data[HOVER_CMD_MOTORCTRL_SIZE];
    int16_t targetLeft = hoverboardWantedStatus.left.target, targetRight = hoverboardWantedStatus.right.target;
    memset(data, 0, sizeof(data));
    data[0] = (targetLeft & 0xFF00) >> 8;
    data[1] = targetLeft & 0x00FF;
    data[3] = (targetRight & 0xFF00) >> 8;
    data[4] = targetRight & 0x00FF;

    sendToHover(HOVER_CMD_MOTORCTRL, data, sizeof(data));

}
void hoverStartDemo() {
    hoverboardWantedStatus.left.target = 12;
    hoverboardWantedStatus.right.target = 12;
    hoverBoardSendMotorCommand();
}

void hoverStopDemo() {
    hoverboardWantedStatus.left.target = 0;
    hoverboardWantedStatus.right.target = 0;
}
uint32_t lastSentPing = millis();
uint32_t lastReceivedESPNowMessage = millis();


void recCallback(kisc::protocol::espnow::KiSCMessage message) {
//    Serial.println("Received message");
    if (message.command == kisc::protocol::espnow::Command::MotorControl) {
        if (xSemaphoreTake(hoverboardStatusMutex, portMAX_DELAY) == pdTRUE) {
            hoverboardWantedStatus.left.target = message.motorControl.left.pwm;
            hoverboardWantedStatus.right.target = message.motorControl.right.pwm;
            hoverboardWantedStatus.left.enabled = message.motorControl.left.enable;
            hoverboardWantedStatus.right.enabled = message.motorControl.right.enable;
            if (message.motorControl.poweroff == 1) {
                if ((hoverboardStatus.state != hoverBoardOffline) &&
                    (hoverboardStatus.state != hoverBoardStopping) &&
                    (hoverboardStatus.state != hoverBoardError)) {
                    shutdownHoverboard();
                }
            } else {
                hoverBoardSendMotorCommand();
            }
            xSemaphoreGive(hoverboardStatusMutex);
        }
        lastReceivedESPNowMessage = millis();
    }
}

void setup() {
    hoverboardStatusMutex = xSemaphoreCreateMutex();
    hoverboardReceiveBufferMutex = xSemaphoreCreateMutex();
    // put your setup code here, to run once:
    delay(2000);
    Serial.begin(115200);
    delay(1000);
  // cppcheck-suppress unknownMacro
    DBGINI(&Serial)
    DBGINI(&Serial, ESP32Timestamp::TimestampSinceStart)
  //    DBGINI(&Serial, ESP32Timestamp::TimestampSinceStart)
    DBGLEV(Info)
    DBGSTA
    DBGLOG(Info, "---------------------------------------------------------------"
                "---------")
    DBGLOG(Info, "Enabled debug levels:")
    DBGLOG(Error, "Error")
    DBGLOG(Warning, "Warning")
    DBGLOG(Info, "Info")
    DBGLOG(Verbose, "Verbose")
    DBGLOG(Debug, "Debug")
    DBGLOG(Info, "---------------------------------------------------------------"
               "---------")    
    // Aktuell Autostart!
    hoverboardStatus.state = hoverBoardStarting;
    onKiSCMessageReceived(recCallback);
    initESPNow();
    motorStatusMessage.command = kisc::protocol::espnow::Command::MotorFeedback;
    DBGLOG(Info, "---- Motor Controller ----");
    DBGLOG(Info, "MAC address: %s", WiFi.macAddress().c_str());
    DBGLOG(Info, "Baud rate: %d", HOVER_SERIAL_BAUD);
    HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HOVER_SERIAL_RX, HOVER_SERIAL_TX);
    HoverSerial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);

    Serial.enableReboot(true);      // Wichtig für Update
}

void shutdownHoverboard() {
    hoverboardStatus.state = hoverBoardStopping;
    hoverShutdown();
}

void wakeupHoverboard() {
    hoverboardStatus.state = hoverBoardStarting;

    Serial.printf("Wakeup Hoverboard (INPUT)\n");
    HoverSerial.end();
    pinMode(HOVER_SERIAL_RX, INPUT);
    pinMode(HOVER_SERIAL_TX, INPUT);
    delay(10);
    digitalWrite(HOVER_SERIAL_RX, HIGH);
    digitalWrite(HOVER_SERIAL_TX, HIGH);
    delay(500);
    digitalWrite(HOVER_SERIAL_RX, LOW);
    digitalWrite(HOVER_SERIAL_TX, LOW);
    delay(500);
    HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HOVER_SERIAL_RX, HOVER_SERIAL_TX);
    delay(50);
    hoverPing();

}

uint lastSendCommand = millis();
uint32_t demoCount = millis();
uint32_t lastTriedToWakeup = millis();

void loop() {
    hoverReceive();
    loopESPNow();
    if ((hoverboardStatus.state == hoverBoardStarting) ||
        (hoverboardStatus.state == hoverBoardOnline)) {
        if (millis() - lastReceivedMessage > 1000) {
            hoverboardStatus.state = hoverBoardError;
            DBGLOG(Error, "Hoverboard error state");
            motorStatusMessage.motorFeedback.motorboardConnected = false;
        }
    } else if (hoverboardStatus.state == hoverBoardStopping) {
        if (millis() - lastReceivedMessage > 300) {
            hoverboardStatus.state = hoverBoardOffline;
            DBGLOG(Info, "Hoverboard stopped");
        }
    } else if (hoverboardStatus.state == hoverBoardStarting) {
        if (millis() - lastReceivedMessage < 200) {
            hoverboardStatus.state = hoverBoardOnline;
            DBGLOG(Info, "Hoverboard onlne");
            motorStatusMessage.motorFeedback.motorboardConnected = true;
        }
    }
    if (hoverboardStatus.state == hoverBoardError) {
        if ((millis() - lastReceivedByte > 1000) && (millis() - lastTriedToWakeup > 5000)) {
            wakeupHoverboard();
            lastTriedToWakeup = millis();
        }
    }
    delay(5);
    // If there are no messages sent to hoverboard, send a ping to keep the connection alive
    if (millis() - lastSendCommand > HOVER_SEND_INTERVAL) {
        if ((hoverboardStatus.state != hoverBoardError) && (hoverboardStatus.state != hoverBoardOffline) && (hoverboardStatus.state != hoverBoardStopping)) {
            hoverBoardSendMotorCommand();
        }
        lastSendCommand = millis();
    } else {
        if (millis() - lastSentToHover > HOVER_PING_INTERVAL) {
            if (hoverboardStatus.state != hoverBoardOffline) {
                if (hoverboardStatus.state == hoverBoardOnline) {
                    hoverPing();
                }
            }
        }
    }
    if (millis() - lastSentKiSCMessage > 2000) {
        sendKiSCMessage(MAIN_CONTROLLER_MAC, motorStatusMessage);
        messageDirtyFlag = false;
        lastSentKiSCMessage = millis();
    }
#if 0
    if ((millis() - demoCount > 20000) && (hoverboardWantedStatus.left.target != 0)) {
        hoverboardWantedStatus.left.target = 0;
        hoverboardWantedStatus.right.target = 0;
        DBGLOG(Info, "Stopping demo");
        demoCount = millis();

    } else if ((millis() - demoCount > 10000) && (hoverboardWantedStatus.left.target == 0)) {
        hoverboardWantedStatus.left.target = 12;
        hoverboardWantedStatus.right.target = 12;
        DBGLOG(Info, "Starting demo");
    }
#endif
}

#endif
#if 0
uint32_t        serialBaud = HOVER_SERIAL_BAUD;
uint32_t        lastReceivedMessage = millis();
uint32_t        lastReceivedSerial     = millis();
bool            bControllerConnected  = false;
bool            bMotorConnected       = false;
bool            inShutdown = false;
bool            bCommunicationEnabled = true;

SerialCommand  command;
SerialFeedback feedback;
SerialFeedback newFeedback;

#ifdef ESP8266
SoftwareSerial HoverSerial(HOVER_SERIAL_RX, HOVER_SERIAL_TX);        // RX, TX
#else

//HardwareSerial HoverSerial(1);        // RX, TX
#define HoverSerial Serial1
#endif

// Global variables
uint16_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
uint16_t numChecksumErrors = 0;

void recCallback(kisc::protocol::espnow::KiSCMessage message) {
//    Serial.println("Received message");
    if (message.command == kisc::protocol::espnow::Command::MotorControl) {
        lastReceivedMessage = millis();
        if (!bControllerConnected) {
            Serial.printf("Controller reconnected\n");
        }
        bControllerConnected = true;
        inShutdown = false;
        bCommunicationEnabled = true;
//        Serial.println("Motor control message");
        if (1) {
//        if (bMotorConnected) {
            command.start = VALID_HEADER;
            command.buzzer.freq = 0;
            command.buzzer.pattern = 0;
            command.led = false;
            command.poweroff = message.motorControl.poweroff;

            command.standstillAcv = message.motorControl.left.standStillOnHold;
            command.cruiseCtrlAcv = message.motorControl.left.cruiseCtrlEna;
            command.electricBrakeAmount = message.motorControl.left.electricBrakeFactor;


            command.left.enable = message.motorControl.left.enable;
            command.left.pwm = 60;
            command.left.ctrlTyp = FieldOrientedControl;
            command.left.ctrlMod = Torque;
            command.left.iMotMax = 10;
            command.left.iDcMax = 10;
            command.left.nMotMax = 1000;
            command.left.fieldWeakMax = message.motorControl.left.fieldWeakMax;
            command.left.phaseAdvMax = 0;
            command.left.cruiseCtrlEna = message.motorControl.left.cruiseCtrlEna;
            command.left.nCruiseMotTgt = message.motorControl.left.cruiseMotTgt;
            command.right.enable = message.motorControl.right.enable;
            command.right.pwm = 60;
            command.right.ctrlTyp = FieldOrientedControl;
            command.right.ctrlMod = Torque;
            command.right.iMotMax = 10;
            command.right.iDcMax = 10;
            command.right.nMotMax = 1000;
            command.right.fieldWeakMax = message.motorControl.right.fieldWeakMax;
            command.right.phaseAdvMax = 0;
            command.right.cruiseCtrlEna = message.motorControl.right.cruiseCtrlEna;
            command.right.nCruiseMotTgt = message.motorControl.right.cruiseMotTgt;
            command.checksum = VALID_HEADER;
//            command.checksum = calculateCommandChecksum(command);
//            if (bCommunicationEnabled) {
                Serial.printf("<<<\n");
#if 0
//#if DEBUG_COMM_SERIAL_SEND
                Serial.printf("<<< ");
                // Printout bytes from feedback as hex characters
                for (int i = 0; i < sizeof(SerialCommand); i++) {
                    Serial.printf("%02X ", ((byte *)&command)[i]);
                }
                Serial.printf("\n");
#endif                
                HoverSerial.write((byte *)&command, sizeof(SerialCommand));
#if DEBUG_COMM_SERIAL_SEND
            } else {
                Serial.printf("Communication disabled\n");
#endif                
//            }
        
        }
    }
}

SerialCommand commandSend;
void SendHoverboardPing() {
    int error = HoverSerial.getWriteError();
    if (error) {
        Serial.printf("Serial write error %d\n", error);
        HoverSerial.clearWriteError();
    } else {
//        Serial.printf("Sending ping\n");
//        SerialCommand command;
        commandSend.start = VALID_HEADER;
        commandSend.buzzer.freq = 4;
        commandSend.buzzer.pattern = 5;
        commandSend.led = false;
        commandSend.poweroff = false;
        commandSend.left.enable = true;
        commandSend.left.pwm = 20;
        commandSend.right.enable = false;
        commandSend.checksum = calculateCommandChecksum(command);
        HoverSerial.write((byte *)&commandSend, sizeof(SerialCommand));
        HoverSerial.flush();
    }
}

uint16_t succeededPackets = 0;

void ReceiveHoverboard() {
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
#if DEBUG_COMM_SERIAL_RECEIVE
//        Serial.printf(".");
//        Serial.printf("%02X ", incomingByte);
#endif
    }
    else {
        return;
    }

    if ((bufStartFrame == VALID_HEADER) && (idx == 0)){  // Initialize if new data is detected
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
            if (!bMotorConnected) {
                Serial.printf("Motor reconnected\n");
            }
            bMotorConnected = true;
            lastReceivedSerial = millis();
            // Copy the new data to the Feedback struct
            memcpy(&feedback, &newFeedback, sizeof(SerialFeedback));
#if DEBUG_COMM_SERIAL_RECEIVE
            // Printout bytes from feedback as hex characters
//            for (int i = 0; i < sizeof(SerialFeedback); i++) {
//                Serial.printf("%02X ", ((byte *)&feedback)[i]);
//            }
//            Serial.printf("\n");
            Serial.printf("  ==> Bat: %4.2f V Temp: %4.1f °C ", feedback.batVoltage / 100.0, feedback.boardTemp / 10.0);
            Serial.printf(" Cruise: %d StandStill: %d EBrake: %d ", feedback.cruiseCtrlAcv, feedback.standstillAcv, feedback.electricBrakeAmount);
            Serial.printf("    ==> Left  motor: Angle: %4d Speed: %4d Error: %1d PWM: %5d\n",
                          feedback.left.angle, feedback.left.speed, feedback.left.error, feedback.left.chops);
            if (feedback.left.chops > 0) {
                Serial.printf("\n\n\n");
                Serial.printf("Success at %d baud\n", serialBaud);
                Serial.printf("\n\n\n");
                Serial.printf("  ==> Bat: %4.2f V Temp: %4.1f °C ", feedback.batVoltage / 100.0, feedback.boardTemp / 10.0);
                Serial.printf(" Cruise: %d StandStill: %d EBrake: %d ", feedback.cruiseCtrlAcv, feedback.standstillAcv, feedback.electricBrakeAmount);
                Serial.printf("    ==> Left  motor: Angle: %4d Speed: %4d Error: %1d PWM: %5d\n",
                              feedback.left.angle, feedback.left.speed, feedback.left.error, feedback.left.chops);
                Serial.printf("    ==> Right motor: Angle: %4d Speed: %4d Error: %1d PWM: %5d\n",
                            feedback.right.angle, feedback.right.speed, feedback.right.error, feedback.right.chops);
                while (1) {
                    delay(1000);
                }
            }

//            Serial.printf("    ==> Right motor: Angle: %4d Speed: %4d Error: %1d PWM: %5d\n",
//                            feedback.right.angle, feedback.right.speed, feedback.right.error, feedback.right.chops);

#endif
            memset(&newFeedback, 0, sizeof(SerialFeedback));
            succeededPackets++;
            numChecksumErrors=0;
            if ((succeededPackets % 100) == 0) {
                Serial.printf("... Success: %d at %d baud...\n", succeededPackets, serialBaud);
            }
        } else {
            HoverSerial.flush();
//            Serial.printf("Checksum error\n");
            numChecksumErrors++;
            if (((numChecksumErrors-succeededPackets) > 10) || (succeededPackets > 100)) {
                serialBaud+=HOVER_SERIAL_BAUD_INC;
                if (serialBaud > HOVER_SERIAL_BAUD_MAX) {
                    serialBaud = HOVER_SERIAL_BAUD;
                }
                if (succeededPackets > 0) {
                    Serial.printf("Success: %d Increasing baud rate to %d\n", succeededPackets, serialBaud);
                }
                HoverSerial.updateBaudRate(serialBaud);
                HoverSerial.flush();
                delay(100);
                numChecksumErrors = 0;
                succeededPackets = 0;
            }
        }
        idx = 0;
        bufStartFrame = 0x0000;
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
    delay(2000);
  // put your setup code here, to run once:

    HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HOVER_SERIAL_RX, HOVER_SERIAL_TX);
    HoverSerial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);
    Serial.begin(115200);
    Serial.enableReboot(true);
#if 0
    onKiSCMessageReceived(recCallback);
    initESPNow();
#endif    
    Serial.printf("\n\n---- Motor Controller ----\n");
    Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());
}

void ShutdownMotors() {
    inShutdown = true;
}

uint32_t lastSentPing = millis();

void loop() {
    // Check for new received data
    ReceiveHoverboard();
    if (millis() - lastSentPing > 50) {
        SendHoverboardPing();
        lastSentPing = millis();
    }    
#if 0    
    loopESPNow();
#endif    
//    sendFeedback();
    if (millis() - lastReceivedMessage > 1000) {
        if (bControllerConnected) {
            Serial.printf("Controller not connected\n");
            bControllerConnected = false;
            ShutdownMotors();
        }
    }
    if (millis() - lastReceivedSerial > 1000) {
        if (bMotorConnected) {
            Serial.printf("Motor not connected\n");
            bMotorConnected = false;
        }
    }
    delay(1);
    if (millis() - lastHeartbeat > 2000) {
//        Serial.printf("Sending heartbeat\n");
        sendHeartbeat();
        lastHeartbeat = millis();
        if (!bMotorConnected) {
//            Serial.printf("Motor not connected\n");
            sendFeedback();
        }
    }
#if 0    
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
#if DEBUG_COMM_SERIAL_SEND
                Serial.printf("<<< ");
                // Printout bytes from feedback as hex characters
                for (int i = 0; i < sizeof(SerialCommand); i++) {
                    Serial.printf("%02X ", ((byte *)&command)[i]);
                }
                Serial.printf("\n");
#endif                
                HoverSerial.write((byte *)&command, sizeof(SerialCommand));
                bCommunicationEnabled = false;
            }
        }
        sendFeedback();
    }
#endif    
 }
 #endif