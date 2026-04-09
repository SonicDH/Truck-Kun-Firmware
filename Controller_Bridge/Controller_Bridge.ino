/*****************************************************************************************
   KEI TRUCK — Controller Bridge
   ---------------------------------------------------------
   This file is the firmware for the bluetooth game controller bridge.
   - This must be run on an microcontroller supported by BluePad32 (https://github.com/ricardoquesada/bluepad32)
   - This particular implementation was for an ESP32-S3 with an OLED, but you can just remove the OLED code.
   - You NEED to set your controller's BT Address in the allowlist (Line 33)

******************************************************************************************/

#include <Bluepad32.h>
#include <uni.h>
#include <U8g2lib.h>
#include "radio_command.h"

// HC-12 pins
#define HC12_TX_PIN  1
#define HC12_RX_PIN  2
#define HC12_SET_PIN 8

// OLED display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);
int xOffset = 28;
int yOffset = 24;

ControllerPtr myControllers[1];

// Configuration
const int DEADZONE = 50;
const unsigned long TRANSMIT_INTERVAL = 11; // ~33Hz

// Allowlist
const char* allowedDevices[] = {"EC:83:50:EE:99:82", "ec:83:50:ee:99:82"}; //set as the BT mac address for your controller to prevent it from pairing with ANY controller.
const int numAllowedDevices = sizeof(allowedDevices) / sizeof(allowedDevices[0]);

// Command packet
radio_command cmd;

// Timing
unsigned long lastTransmit = 0;

// State
bool lightsOn = false;
bool eStopActive = false;
bool speed_restriction_on = true;

// Stupid lights button debounce
unsigned long lastButtonPress = 0;
const unsigned long DebounceInterval = 150;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void Display(String text) {
    u8g2.clearBuffer();
    u8g2.drawStr(xOffset, yOffset + 25, text.c_str());
    u8g2.sendBuffer();
}

void DisplayStatus(int16_t throttle, int16_t steering, bool lights, bool estop) {
    u8g2.clearBuffer();
    char line1[20];
    char line2[20];
    snprintf(line1, sizeof(line1), "T:%4d S:%4d", throttle, steering);
    snprintf(line2, sizeof(line2), "L:%d E:%d", lights ? 1 : 0, estop ? 1 : 0);
    u8g2.drawStr(xOffset, yOffset + 15, line1);
    u8g2.drawStr(xOffset, yOffset + 30, line2);
    u8g2.sendBuffer();
}

int applyDeadzone(int value, int deadzone) {
    if (abs(value) < deadzone) return 0;
    return value;
}

uint8_t computeChecksum(const radio_command &c) {
    uint8_t x = 0;
    x ^= c.e_stop;
    x ^= (uint8_t)(c.throttle & 0xFF);
    x ^= (uint8_t)((c.throttle >> 8) & 0xFF);
    x ^= (uint8_t)(c.brake_trigger & 0xFF);
    x ^= (uint8_t)((c.brake_trigger >> 8) & 0xFF);
    x ^= c.brake_button;
    x ^= c.front_brake;
    x ^= (uint8_t)(c.steering & 0xFF);
    x ^= (uint8_t)((c.steering >> 8) & 0xFF);
    x ^= c.lights;
    return x;
}

void sendCommand(radio_command &cmd) {
    cmd.magic = 0x4B;
    cmd.checksum = computeChecksum(cmd);
    
    // Serial.printf("SEND seq=%d\n", cmd.sequence);
    Serial1.write((uint8_t*)&cmd, sizeof(cmd));
    //Serial.write((uint8_t*)&cmd, sizeof(cmd));
    
    // Debug
    Serial.printf("TX: thr=%d brk=%d str=%d chk=0x%02X\n",
                  cmd.throttle, cmd.brake_trigger, 
                  cmd.steering, cmd.checksum);
}

void sendEmergencyStop() {
    cmd.e_stop = 1;
    cmd.throttle = 0;
    cmd.brake_trigger = 1023;  // Full brake
    cmd.brake_button = 1;
    cmd.front_brake = 1;
    cmd.steering = 0;
    cmd.lights = 0;
    
    for (int i = 0; i < 5; i++) {
        sendCommand(cmd);
        delay(10);
    }
    
    //Serial.println("EMERGENCY STOP SENT");
    Display("E-STOP!");
}

// ============================================================================
// CONTROLLER ALLOWLIST
// ============================================================================

bool isControllerAllowed(ControllerPtr ctl) {
    ControllerProperties properties = ctl->getProperties();
    char addrStr[18];
    snprintf(addrStr, sizeof(addrStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
             properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);
    
    Serial.printf("Controller: %s\n", addrStr);
    
    for (int i = 0; i < numAllowedDevices; i++) {
        if (strcmp(addrStr, allowedDevices[i]) == 0) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// BLUEPAD32 CALLBACKS
// ============================================================================

void onConnectedController(ControllerPtr ctl) {
    if (!isControllerAllowed(ctl)) {
        Serial.println("Controller rejected");
        ctl->disconnect();
        return;
    }
    
    if (myControllers[0] == nullptr) {
        Serial.println("Controller connected");
        myControllers[0] = ctl;
        eStopActive = false;
        Display("READY");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myControllers[0] == ctl) {
        Serial.println("Controller disconnected");
        myControllers[0] = nullptr;
        sendEmergencyStop();
    }
}

// ============================================================================
// GAMEPAD PROCESSING
// ============================================================================

void processGamepad(ControllerPtr ctl) {
    int rawThrottle = ctl->throttle();   // 0-1023
    int rawBrake = ctl->brake();         // 0-1023
    int rawSteering = ctl->axisX();      // -511 to +512
    
    if (speed_restriction_on) {
        rawThrottle = rawThrottle/6;
        rawBrake = rawBrake/6;
    }
    else {
        rawThrottle = rawThrottle/4;
        rawBrake = rawBrake/4;
    }

    // Apply deadzone to steering
    int steering = applyDeadzone(rawSteering, DEADZONE);
    steering = constrain(steering, -511, 511);
    
    // Apply small deadzone to triggers
    int throttle = applyDeadzone(rawThrottle, 5);
    int brake = applyDeadzone(rawBrake, 5);


    // Handle E-STOP (B button)
    if (ctl->b()) {
        if (!eStopActive) {
            eStopActive = true;
            sendEmergencyStop();
            Display("E-STOP");
        }
        delay(200);
    }
    
    // Clear E-STOP (Start button)
    if (ctl->miscStart()) {
        eStopActive = false;
        //Display("ACTIVE");
        delay(200);
    }
    
    // Lights toggle (A button)
    if (ctl->x()) {
        if (millis() - lastButtonPress > DebounceInterval) {
            lightsOn = !lightsOn;
            //Display(lightsOn ? "LIGHTS ON" : "LIGHTS OFF");
            lastButtonPress = millis();  // Prevent spam
    }
        
    }

    // Speed restriction toggle (share button)
    if (ctl->miscSelect()) {
        if (millis() - lastButtonPress > DebounceInterval) {
            speed_restriction_on  = !speed_restriction_on;
            //Display(speed_restriction_on ? "SIMP" : "CHAD");
            lastButtonPress = millis();  // Prevent spam
    }
        
    }

    
    // Handbrake (X button)
    bool handbrake = ctl->a();
    bool burnout = ctl->l1();
    
    // Build packet
    cmd.e_stop = eStopActive ? 1 : 0;
    cmd.throttle = eStopActive ? 0 : (int16_t)throttle;
    cmd.brake_trigger = eStopActive ? 1023 : (int16_t)brake;
    cmd.brake_button = (handbrake && !eStopActive) ? 1 : 0;
    cmd.front_brake = (burnout && !eStopActive) ? 1 : 0;
    cmd.steering = eStopActive ? 0 : (int16_t)steering;
    cmd.lights = lightsOn ? 1 : 0;
    


    // Update display
    // DisplayStatus(cmd.throttle, cmd.steering, lightsOn, eStopActive);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    //Serial.println("\n=== KEI TRUCK CONTROLLER ===");
    
    // HC-12 setup
    pinMode(HC12_SET_PIN, OUTPUT);
    digitalWrite(HC12_SET_PIN, HIGH);  // Transmission mode
    Serial1.begin(38400, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);
   // Serial.println("HC-12 initialized at 9600 baud");
    
    // Bluepad32
    //Serial.printf("Bluepad32: %s\n", BP32.firmwareVersion());
    BP32.setup(&onConnectedController, &onDisconnectedController);
    
    // OLED
    u8g2.begin();
    u8g2.setContrast(255);
    u8g2.setBusClock(400000);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    Display("WAITING");
    lastButtonPress = millis();
    //Serial.println("Ready");
}

void loop() {
    // Update controller state
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }
    
    // Transmit at fixed rate
    unsigned long now = millis();
    if (now - lastTransmit >= TRANSMIT_INTERVAL) {
        lastTransmit = now;
        
        if (myControllers[0] != nullptr && myControllers[0]->isConnected()) {
            sendCommand(cmd);
        } else {
            // No controller - send E-STOP
            cmd.e_stop = 1;
            cmd.throttle = 0;
            cmd.brake_trigger = 1023;
            cmd.brake_button = 1;
            cmd.front_brake = 1;
            cmd.steering = 0;
            cmd.lights = 0;
            sendCommand(cmd);
        }
    }
    
    // delay(10);
}