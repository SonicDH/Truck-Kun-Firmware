/*****************************************************************************************
   KEI TRUCK — Lighting Firmware
   ---------------------------------------------------------
   This is the SIMPLEST firmware. If you wanted a jumping off point for a new kind of device,
   start with this. It literally just has ONE controllable parameter: Lights on or off. 
   It's that simple.
   The only condition where it will call an ESTOP is if it doesn't get a packet from the master
   within the 300ms timeout window. It's lights. How could it fail catastrophically??

******************************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <hardware/sync.h>
#include "i2c_packet_structs.h" 
#include <Adafruit_NeoPixel.h>

static const uint8_t DEVICE_I2C_ADDRESS = 0x30;   // EX: rear axle = 0x10, front axle = 0x11
static const uint32_t PACKET_TIMEOUT_MS = 300;
static const uint8_t POLL_COMMAND = 0xEE;


struct SharedState {
    lightpacket   latest_packet;    // Last received axlepacket from master
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

// Hardware pins for lights
#define STATUS_LIGHT 16
Adafruit_NeoPixel pixel(1, STATUS_LIGHT, NEO_GRB + NEO_KHZ800);

#define HEADLIGHT_PIN 1

// Called once from Core 0
void deviceSetup() {
    // Example motor driver pin setup:
    pixel.begin();
    // Start safe: Turn on lights to show that, at least, things are happening, even if a packet turns them immediately off after.
    pixel.setPixelColor(0, pixel.Color(150, 150, 150)); 
    pixel.show();
    pinMode(HEADLIGHT_PIN, OUTPUT);
    digitalWrite(HEADLIGHT_PIN, HIGH);
    delay(500); //keep them on just long enough to see
    digitalWrite(HEADLIGHT_PIN, LOW);
    Serial.println("Lights Setup complete.");
    pixel.setPixelColor(0, pixel.Color(0, 0, 0)); 
    pixel.show();
}


// Called when a fresh packet arrives AND we are not in ESTOP.
void handleNewPacket(const lightpacket& pkt) {
    if (estop_status) { //turn off the estop red light if it's on and reset estop status flag
        pixel.setPixelColor(0, pixel.Color(0, 0, 0)); 
        pixel.show();
    }
    // This struct currently only has one parameter and it either 1 or 0
    if (pkt.lights) {
        digitalWrite(HEADLIGHT_PIN, HIGH);
        pixel.setPixelColor(0, pixel.Color(255, 255, 255)); 
        pixel.show();
        Serial.println("light on");
        return;
    }
    else {
        digitalWrite(HEADLIGHT_PIN, LOW);
        pixel.setPixelColor(0, pixel.Color(0, 0, 0)); 
        pixel.show();
        Serial.println("light off");
    }
    
}


// Called when ESTOP is active for ANY reason:
void handleEStop() {
    estop_status = true;
    Serial.println("ESTOP");
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show(); 
}


// Called every loop (Core 0). Good for status LEDs or sensor checks.
void deviceLoop(bool estopActive) {
    // Optional heartbeat or status indicator
}



/*****************************************************************************************
   STEP 4 — I²C CALLBACKS (Core 1)
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
    // Check Packet type
    if ((pkt.device_type & 0xF0) != (DEVICE_I2C_ADDRESS & 0xF0)) {
        while (Wire1.available()) Wire1.read();
        return;
    }
    
    //devices require EXACT size match
    if (numBytes != sizeof(lightpacket)) {
        while (Wire1.available()) Wire1.read();
        return;
    }

    // Read packet
    lightpacket pkt;
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

    Serial.println("\n=== Light Controller ===");

    lockPtr = spin_lock_instance(5);

    // Initialize shared state to safe defaults
    lock_shared();
    memset(&shared_state.latest_packet, 0, sizeof(lightpacket));
    shared_state.last_packet_ms     = millis();
    shared_state.estop_from_master  = true;   // safe by default
    shared_state.estop_from_device  = false;
    unlock_shared();

    deviceSetup();

    multicore_launch_core1(core1_entry);

    Serial.println("[Core0] Setup complete.");
}


void loop() {
    // Snapshot shared state
    lightpacket pkt;
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
        Serial.println(timed_out);
        Serial.println(estop_device);
        Serial.println(estop_master);
        handleEStop();
    } else {
        handleNewPacket(pkt);
    }

    deviceLoop(estop_effective);
}
