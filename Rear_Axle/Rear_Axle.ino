/*****************************************************************************************
   KEI TRUCK — Rear Axle Firmware
   ---------------------------------------------------------
   This is setup too drive two motors simultaneously at the same speed (rather than independant control).
   It's setup assuming the motor controllers have direction, speed (PWM), brake, and speed sensing pins.
   The exact controller it's designed around is the RioRand 250W BLDC Controller.

******************************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <hardware/sync.h>
#include "i2c_packet_structs.h"
#include <Adafruit_NeoPixel.h>

const bool SPEED_FAULT_CHECK_ENABLED = false; // FALSE for benchtesting
static const uint8_t DEVICE_I2C_ADDRESS = 0x10;   // EX: rear axle = 0x10, front axle = 0x11
static const uint32_t PACKET_TIMEOUT_MS = 300;
static const uint8_t POLL_COMMAND = 0xEE;


struct SharedState {
    axlepacket   latest_packet;    // Last received axlepacket from master
    uint32_t     last_packet_ms;   // millis() timestamp of last packet
    bool         estop_from_master; // packet.estop
    bool         estop_from_device; // your local faults
};

SharedState shared_state;
spin_lock_t* lockPtr = nullptr;

// Helpers
inline void lock_shared()   { spin_lock_unsafe_blocking(lockPtr); }
inline void unlock_shared() { spin_unlock_unsafe(lockPtr); }
bool estop_status = false;
bool brake_state = false;
bool direction_state = 0;
uint8_t pwm_state = 0;

// Hardware pins for status light
#define HEADLIGHT_PIN 16
Adafruit_NeoPixel pixel(1, HEADLIGHT_PIN, NEO_GRB + NEO_KHZ800);

// Constants
const unsigned long SPEED_TIMEOUT = 500000;       // Time used to determine wheel is not spinning
const double WHEEL_DIAMETER_IN = 6.5;            // Motor wheel diamater (inches)
const double WHEEL_CIRCUMFERENCE_IN = 22.25;     // Motor wheel circumference (inches)
const double WHEEL_DIAMETER_CM = 16.5;           // Motor wheel diamater (centimeters)
const double WHEEL_CIRCUMFERENCE_CM = 56.5;      // Motor wheel circumference (centimeters)

// Pin Declarations
//Motor 1
const int M1_PIN_DIR = 6;      // Motor direction signal
const int M1_PIN_BRAKE = 5;    // Motor brake signal (active low)
const int M1_PIN_PWM = 3;      // PWM motor speed control
const int M1_PIN_SPEED = 4;   // SC Speed Pulse Output from RioRand board

//Motor 2
const int M2_PIN_DIR = 29;      // Motor direction signal
const int M2_PIN_BRAKE = 28;    // Motor brake signal (active low)
const int M2_PIN_PWM = 27;      // PWM motor speed control
const int M2_PIN_SPEED = 26;   // SC Speed Pulse Output from RioRand board

// Called once from Core 0
void deviceSetup() {
    pixel.begin();
    // Start safe: Turn on lights to show that, at least, things are happening, even if a packet turns them immediately off after.
    pixel.setPixelColor(0, pixel.Color(150, 150, 150)); 
    pixel.show();
    // Set pin directions
    pinMode(M1_PIN_SPEED, INPUT);
    pinMode(M1_PIN_PWM, OUTPUT);
    pinMode(M1_PIN_BRAKE, OUTPUT);
    pinMode(M1_PIN_DIR, OUTPUT);
    pinMode(M2_PIN_SPEED, INPUT);
    pinMode(M2_PIN_PWM, OUTPUT);
    pinMode(M2_PIN_BRAKE, OUTPUT);
    pinMode(M2_PIN_DIR, OUTPUT);
    
    // Set initial pin states
    
    commandMotors(0, 0, 0);

    Serial.println("Axle Setup complete.");
}

void commandMotors(bool brake, bool direction, uint8_t speed) { 
    brake_state = brake;
    direction_state = direction;
    pwm_state = speed;
    // inputs: Brake: 1 or 0, Direction: 1 or 0, speed 0-255 
    // Motor 1
    digitalWrite(M1_PIN_BRAKE, brake);
    digitalWrite(M1_PIN_DIR, direction);
    analogWrite(M1_PIN_PWM, speed);

    //Motor 2 - should turn opposite direction of Motor 1
    digitalWrite(M2_PIN_BRAKE, brake);
    digitalWrite(M2_PIN_DIR, !direction);
    analogWrite(M2_PIN_PWM, speed);
}


// Called when a fresh packet arrives AND we are not in ESTOP.
void handleNewPacket(const axlepacket& pkt) {
    Serial.printf("Brake:%d , Direction:%d, PWM:%d\n", bool(pkt.brake), bool(pkt.direction), constrain(pkt.pwm, 0, 255));
    commandMotors(bool(pkt.brake),bool(pkt.direction),constrain(pkt.pwm, 0, 255));
}


// Called when ESTOP is active for ANY reason:
void handleEStop() {
    estop_status = true;
    commandMotors(1,0,0);
    pixel.setPixelColor(0, pixel.Color(255, 0, 0)); 
    pixel.show();
    Serial.println("ESTOP");
}


// Called every loop (Core 0). Good for status LEDs or sensor checks.
void deviceLoop(bool estopActive) {
    ReadSpeed();
    if (SPEED_FAULT_CHECK_ENABLED && brake_state && pwm_state > 100 && _rpm[0] == 0 && _rpm[1] == 0)
        {
            shared_state.estop_from_device = true;
        }


}



// Reads the speed from the input pin and calculates RPM and MPH
// Monitors the state of the input pin and measures the time (µs) between pin transitions
void ReadSpeed()
{
    static bool lastState[2] = {false, false};           // Track state for both motors
    static unsigned long last_uS[2] = {0, 0};            // Last transition time for each motor
    static unsigned long timeout_uS[2] = {0, 0};         // Timeout timer for each motor
    
    // Array of pins to check
    int speedPins[2] = {M1_PIN_SPEED, M2_PIN_SPEED};
    
    // Loop through both motors
    for (int i = 0; i < 2; i++)
    {
        // Read the current state of the input pin for this motor
        bool state = digitalRead(speedPins[i]);

        // Check if the pin has changed state
        if (state != lastState[i])
        {
            // Calculate how long has passed since last transition
            unsigned long current_uS = micros();
            unsigned long elapsed_uS = current_uS - last_uS[i];

            // Calculate the frequency of the input signal
            double period_uS = elapsed_uS * 2.0;
            double freq = (1 / period_uS) * 1E6;

            // Calculate the RPM
            double rpm = freq / 45 * 60;

            // If RPM is excessively high then ignore it
            if (rpm > 5000) rpm = 0;

            // Calculate the miles per hour (mph)
            double mph = (WHEEL_CIRCUMFERENCE_IN * rpm * 60) / 63360; 

            // Store the results for this motor
            _rpm[i] = rpm;
            _mph[i] = mph;
            _freq[i] = freq;
      
            // Save the last state and next timeout time
            last_uS[i] = current_uS;
            timeout_uS[i] = last_uS[i] + SPEED_TIMEOUT;
            lastState[i] = state;
        }
        // If too long has passed then the wheel has probably stopped
        else if (micros() > timeout_uS[i])
        {
            _freq[i] = 0;
            _rpm[i] = 0;
            _mph[i] = 0;
            last_uS[i] = micros();
        }
    }
}


/*****************************************************************************************
   STEP 4 — I2C CALLBACKS (Core 1)
   --------------------------------
   DO NOT PUT BLOCKING CODE HERE.
   DO NOT do slow operations here.
   DO NOT do Serial.print inside steady callbacks.
******************************************************************************************/

volatile bool last_command_was_poll = false;

// When master WRITES to this device
void onI2CReceive(int numBytes) {
    if (numBytes <= 0) return;

    last_command_was_poll = false;

    // If it's a single byte, it may be a poll command
    if (numBytes == 1) {
        uint8_t cmd = Wire1.read();
        if (cmd == POLL_COMMAND) {
            last_command_was_poll = true;
        }
        return;
    }

    // Axle devices require EXACT size match
    if (numBytes != sizeof(axlepacket)) {
        while (Wire1.available()) Wire1.read();
        return;
    }

    // Read packet
    axlepacket pkt;
    int r = Wire1.readBytes((char*)&pkt, sizeof(pkt));
    if (r != sizeof(pkt)) {
        while (Wire1.available()) Wire1.read();
        return;
    }

    uint32_t now = millis();

    // Update shared state safely
    lock_shared();
    shared_state.latest_packet    = pkt;
    shared_state.last_packet_ms   = now;
    shared_state.estop_from_master = (pkt.estop != 0);
    unlock_shared();
}


// When master READS from this device
void onI2CRequest() {
    // Report ONLY device-side ESTOP to master
    Wire1.write(shared_state.estop_from_device ? 1 : 0);
}



/*****************************************************************************************
   STEP 5 — CORE 1 ENTRY POINT
******************************************************************************************/

void core1_entry() {
    Wire1.setSDA(10);
    Wire1.setSCL(11);
    Wire1.begin(DEVICE_I2C_ADDRESS);
    Wire1.onReceive(onI2CReceive);
    Wire1.onRequest(onI2CRequest);

    Serial.println("[Core1] I2C active.");

    while (true) {
        tight_loop_contents();  // keep latency low
    }
}



/*****************************************************************************************
   STEP 6 — CORE 0 MAIN LOGIC
******************************************************************************************/

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== Rear Axle Controller ===");

    lockPtr = spin_lock_instance(5);

    // Initialize shared state to safe defaults
    lock_shared();
    memset(&shared_state.latest_packet, 0, sizeof(axlepacket));
    shared_state.last_packet_ms     = millis();
    shared_state.estop_from_master  = false;   // safe by default
    shared_state.estop_from_device  = false;
    unlock_shared();

    deviceSetup();

    multicore_launch_core1(core1_entry);

    Serial.println("[Core0] Setup complete.");
}


void loop() {
    // Snapshot shared state
    axlepacket pkt;
    bool estop_master, estop_device;
    uint32_t lastPktMs;

    lock_shared();
    pkt           = shared_state.latest_packet;
    estop_master  = shared_state.estop_from_master;
    estop_device  = shared_state.estop_from_device;
    lastPktMs     = shared_state.last_packet_ms;
    unlock_shared();

    // Watchdog timeout
    uint32_t now = millis();
    bool timed_out = ((now - lastPktMs) > PACKET_TIMEOUT_MS);

    // Effective ESTOP = any safety condition active
    bool estop_effective = estop_master || estop_device || timed_out;

    // Apply logic
    if (estop_effective) {
        handleEStop();
    } else {
        pixel.setPixelColor(0, pixel.Color(0, 0, 0)); 
        pixel.show();
        handleNewPacket(pkt);
    }

    deviceLoop(estop_effective);
}
