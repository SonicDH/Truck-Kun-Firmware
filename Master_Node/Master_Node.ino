#include <Arduino.h>
#include <Wire.h>
#include <hardware/sync.h>
#include "radio_command.h"
#include "i2c_packet_structs.h"
#include <Adafruit_NeoPixel.h>
#define HC12_SET_PIN 2

// ============================================================
//  NEW — Shared State + Spinlock
// ============================================================

struct SharedState {
    axlepacket rear_pkt;
    axlepacket front_pkt;
    steeringpacket steer_pkt;
    lightpacket light_pkt;

    uint8_t estop;               // unified estop state (0/1)
    uint16_t estop_flags;        // Why we stopped
    uint32_t core0_last_update;  // heartbeat timestamp from Core0
};

SharedState shared;
spin_lock_t *lockPtr;   // pointer to HW spinlock

// toggle for device-side fault polling
bool DEVICE_ESTOP_POLLING = true;

// watchdog for Core0
const uint32_t CORE0_TIMEOUT_MS = 120;   // around 2× your packet timeout

// Hardware pins for status light
#define HEADLIGHT_PIN 16
Adafruit_NeoPixel pixel(1, HEADLIGHT_PIN, NEO_GRB + NEO_KHZ800);


// =============================================================
// Original variables (unchanged)
// =============================================================
radio_command cmd;
uint8_t targetPWM = 0;
int currentPWM = 0;
const int RAMP_RATE = 10;
uint8_t dir = 1;                // 1 = forward, 0 = reverse
uint8_t targetDir = 1;
unsigned long dirChangeTimestamp = 0;
bool dirLockout = false;

const uint16_t DIR_LOCKOUT_TIME = 400;  // ms

unsigned long lastPacketTime = 0;
unsigned long packetCount = 0;
unsigned long totalLatency = 0;

unsigned long lastGoodPacket = 0;
const unsigned long WATCHDOG_TIMEOUT = 150;

bool last_estop_state = false;


// I2C addresses
#define Rear_Axle  0x10
#define Front_Axle 0x11
#define Steering   0x20
#define Lights     0x30

// pack objects (for convenience)
axlepacket axlepkt;
steeringpacket steeringpkt;
lightpacket lightpkt;


// =============================================================
// Spinlock helpers
// =============================================================
inline void lock() {
    spin_lock_unsafe_blocking(lockPtr);   // atomic lock
}

inline void unlock() {
    spin_unlock_unsafe(lockPtr);          // atomic unlock
}


// =============================================================
//  Core 1 Device ESTOP poll helper (optional)
// =============================================================

enum EstopReason : uint16_t {
    ESTOP_NONE              = 0,
    ESTOP_RADIO_COMMAND     = 1 << 0,  // cmd.e_stop asserted
    ESTOP_RADIO_TIMEOUT     = 1 << 1,  // WATCHDOG_TIMEOUT
    ESTOP_CORE0_TIMEOUT     = 1 << 2,  // core0 heartbeat lost
    ESTOP_REAR_DEVICE       = 1 << 3,
    ESTOP_FRONT_DEVICE      = 1 << 4,
    ESTOP_STEERING_DEVICE   = 1 << 5,
    ESTOP_CHECKSUM_FAULT    = 1 << 6,
};

bool poll_estop(uint8_t addr) {
    Wire1.beginTransmission(addr);
    Wire1.write(0xEE);   // example "status request" command — adjust for real FW 
    if (Wire1.endTransmission() != 0) return false;

    Wire1.requestFrom(addr, (uint8_t)1);
    if (Wire1.available()) {
        uint8_t est = Wire1.read();
        return est != 0;
    }
    return false;
}


// ============================================================================
// Compute RC packet checksum
// ============================================================================
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


// ============================================================================
// CONTROL LOGIC
// ============================================================================

//  MODIFIED: logic builds packets, but NO LONGER calls sendpacket()

void axle_logic() {
    int16_t brake = map(cmd.brake_trigger, 0, 1023, 0, 255);
    int16_t gas   = map(cmd.throttle,       0, 1023, 0, 255);
    int16_t overall = gas - brake;   // positive = forward, negative = reverse

    // determine what the commanded direction SHOULD be
    if (overall > 10)      targetDir = 1;
    else if (overall < -10) targetDir = 0;

    unsigned long now = millis();

    // ============================================================
    // 1. Direction change detection → enter lockout so you don't shock the motor mounts
    // ============================================================
    if (targetDir != dir && !dirLockout) {
        dirLockout = true;
        dirChangeTimestamp = now;
        targetPWM = 0;             // force motor to zero immediately
    }

    // ============================================================
    // 2. Handle lockout delay
    // ============================================================
    if (dirLockout) {
        // Hold motor at zero until delay expires
        targetPWM = 0;

        if (now - dirChangeTimestamp >= DIR_LOCKOUT_TIME) {
            // lockout expired → apply new direction
            dir = targetDir;
            dirLockout = false;
        }
    }
    else {
        // ============================================================
        // 3. Normal motion logic AFTER lockout
        // ============================================================
        if (overall > 10 && dir == 1) {
            targetPWM = constrain(overall, 0, 255);
        }
        else if (overall < -10 && dir == 0) {
            targetPWM = constrain(abs(overall) / 2, 0, 127);
        }
        else {
            targetPWM = 0;
        }
    }

    // ============================================================
    // 4. Smooth ramping
    // ============================================================
    if (currentPWM < targetPWM) currentPWM = min(currentPWM + RAMP_RATE, (int)targetPWM);
    else if (currentPWM > targetPWM) currentPWM = max(currentPWM - RAMP_RATE, (int)targetPWM);

    // build rear axle packet
    axlepkt.device_type = 0x10;
    axlepkt.estop = cmd.e_stop;
    axlepkt.brake = cmd.brake_button;
    axlepkt.direction = dir;
    axlepkt.pwm = currentPWM;

    // build front axle (no brake button and speed compensation)
    axlepacket frontpkt = axlepkt;
    frontpkt.brake = cmd.front_brake;
    frontpkt.pwm = (uint8_t)(currentPWM * 0.87); // 759/831 compensation - If your motors match you wont need this. Mine don't tho.

    // copy into shared state atomically
    lock();
    shared.rear_pkt = axlepkt;
    shared.front_pkt = frontpkt;
    unlock();
}


void steering_logic() {
    steeringpkt.device_type = 0x20;
    steeringpkt.estop = cmd.e_stop;
    steeringpkt.position = cmd.steering;

    lock();
    shared.steer_pkt = steeringpkt;
    unlock();
}


void lights_logic() {
    lightpkt.device_type = 0x30;
    lightpkt.estop = cmd.e_stop;
    lightpkt.lights = cmd.lights;

    lock();
    shared.light_pkt = lightpkt;
    unlock();
}


// ============================================================================
// EMERGENCY STOP — modified to write into shared state only
// ============================================================================
void emergency_stop() {
    pixel.setPixelColor(0, pixel.Color(255, 0, 0)); 
    pixel.show();
    axlepacket stop_axle;
    stop_axle.device_type = 0x10;
    stop_axle.estop = 1;
    stop_axle.brake = 1;
    stop_axle.direction = 1;
    stop_axle.pwm = 0;

    steeringpacket stop_steer;
    stop_steer.device_type = 0x20;
    stop_steer.estop = 1;
    stop_steer.position = 0;

    lightpacket stop_light;
    stop_light.device_type = 0x30;
    stop_light.estop = 1;
    stop_light.lights = 0;

    lock();
    shared.rear_pkt  = stop_axle;
    shared.front_pkt = stop_axle;
    shared.steer_pkt = stop_steer;
    shared.light_pkt = stop_light;
    shared.estop = 1;   //  unified estop
    unlock();

    //Serial.println("!!! EMERGENCY STOP !!!");
}

void assert_estop(uint16_t reason) {
    lock();
    if (!shared.estop) {
        shared.estop = 1;
        shared.estop_flags = reason;
    } else {
        shared.estop_flags |= reason;
    }
    unlock();
}


// ============================================================================
// PACKET STATISTICS
// ============================================================================
void measurePacketRate() {
    unsigned long now = millis();
    if (lastPacketTime > 0) {
        unsigned long interval = now - lastPacketTime;
        totalLatency += interval;
        packetCount++;
        if (packetCount % 100 == 0) {
            float avgRate = 1000.0 / (totalLatency / (float)packetCount);
            Serial.printf("Rate: %.1f Hz (%.1f ms avg)\n",
                          avgRate, totalLatency / (float)packetCount);
            totalLatency = 0;
            packetCount = 0;
        }
    }
    lastPacketTime = now;
}


// ============================================================================
// PACKET RECEPTION
// ============================================================================
bool receivePacket() {

    while (Serial1.available() > 0) {
        if (Serial1.read() == 0x4B) {
            unsigned long timeout = millis() + 100;
            while (Serial1.available() < 11) {
                if (millis() > timeout) return false;
            }

            cmd.magic = 0x4B;
            cmd.e_stop = Serial1.read();

            uint8_t thr_lo = Serial1.read();
            uint8_t thr_hi = Serial1.read();
            cmd.throttle = thr_lo | (thr_hi << 8);

            uint8_t brk_lo = Serial1.read();
            uint8_t brk_hi = Serial1.read();
            cmd.brake_trigger = brk_lo | (brk_hi << 8);

            cmd.brake_button = Serial1.read();
            cmd.front_brake = Serial1.read();
            uint8_t str_lo = Serial1.read();
            uint8_t str_hi = Serial1.read();
            cmd.steering = str_lo | (str_hi << 8);

            cmd.lights = Serial1.read();
            cmd.checksum = Serial1.read();

            uint8_t calculated = computeChecksum(cmd);
            if (calculated != cmd.checksum) {
                Serial.printf("✗ Checksum fail: got 0x%02X, calc 0x%02X\n",
                              cmd.checksum, calculated);
                return false;
            }

            return true;
        }
    }
    return false;
}


// ============================================================================
// CORE 1 TASK — I2C dispatcher + safety supervisor
// ============================================================================
void core1_entry() {

    uint32_t lastTick = millis();

    while (true) {

        // run every ~11ms
        if (millis() - lastTick >= 11) {
            lastTick = millis();

            // ----------------------------
            //  1. COPY SHARED PACKETS
            // ----------------------------
            SharedState local;

            lock();
            local = shared;
            unlock();

            // ----------------------------
            //  2. WATCHDOG: Core0 heartbeat
            // ----------------------------
            if (millis() - local.core0_last_update > CORE0_TIMEOUT_MS) {
                local.estop = 1;
                assert_estop(ESTOP_CORE0_TIMEOUT);
            }


            // ----------------------------
            //  3. OPTIONAL DEVICE POLLING
            // ----------------------------
            if (DEVICE_ESTOP_POLLING) {
                bool r  = poll_estop(Rear_Axle);
                bool f  = poll_estop(Front_Axle);
                bool s  = poll_estop(Steering);

                if (r) assert_estop(ESTOP_REAR_DEVICE);
                if (f) assert_estop(ESTOP_FRONT_DEVICE);
                if (s) assert_estop(ESTOP_STEERING_DEVICE);

            }

            // ----------------------------
            //  4. If estop: overwrite packets
            // ----------------------------
            if (local.estop) {
                axlepacket stopA;
                stopA.device_type = 0x10;
                stopA.estop = 1; stopA.brake = 1;
                stopA.direction = 1; stopA.pwm = 0;

                steeringpacket stopS;
                stopS.device_type = 0x20;
                stopS.estop = 1; stopS.position = 0;

                lightpacket stopL;
                stopL.device_type = 0x30;
                stopL.estop = 1; stopL.lights = 0;

                local.rear_pkt  = stopA;
                local.front_pkt = stopA;
                local.steer_pkt = stopS;
                local.light_pkt = stopL;

                // propagate estop into shared state
                lock();
                shared.estop = 1;
                unlock();
            }

            // ----------------------------
            //  5. SEND PACKETS VIA I2C
            // ----------------------------
            Wire1.beginTransmission(Rear_Axle);
            Wire1.write((uint8_t*)&local.rear_pkt, sizeof(axlepacket));
            Wire1.endTransmission();

            Wire1.beginTransmission(Front_Axle);
            Wire1.write((uint8_t*)&local.front_pkt, sizeof(axlepacket));
            Wire1.endTransmission();

            Wire1.beginTransmission(Steering);
            Wire1.write((uint8_t*)&local.steer_pkt, sizeof(steeringpacket));
            Wire1.endTransmission();

            Wire1.beginTransmission(Lights);
            Wire1.write((uint8_t*)&local.light_pkt, sizeof(lightpacket));
            Wire1.endTransmission();
        }
    }
}


// ============================================================================
// SETUP
// ============================================================================
void setup() {
    pixel.begin();
    // Start safe: Turn on lights to show that, at least, things are happening, even if a packet turns them immediately off after.
    pixel.setPixelColor(0, pixel.Color(150, 150, 150)); 
    pixel.show();
    delay(1500);

    pinMode(HC12_SET_PIN, OUTPUT);
    digitalWrite(HC12_SET_PIN, HIGH);

    Wire1.setSDA(10);
    Wire1.setSCL(11);
    Wire1.begin();

    Serial.begin(115200);
    Serial.println("\n=== KEI TRUCK MASTER CONTROLLER (Dual Core) ===");
    Serial1.begin(38400);
    Serial.println("Waiting for packets\n");

    lastGoodPacket = millis();

    lockPtr = spin_lock_instance(5);

    lock();
    shared.estop = 1;
    shared.estop_flags = ESTOP_NONE;
    shared.core0_last_update = millis();
    unlock();


    multicore_launch_core1(core1_entry);
}


// ============================================================================
// CORE 0 — Main Loop (recieve data from RC controller and build packet data)
// ============================================================================
void loop() {
    last_estop_state = false;
    if (receivePacket()) {
        lastGoodPacket = millis();
        //measurePacketRate();

        // run existing logic (build-only)
        axle_logic();
        steering_logic();
        lights_logic();

        // mirror estop into shared state
        lock();
        shared.estop = cmd.e_stop ? 1 : 0;
        shared.core0_last_update = millis();   // heartbeat
        unlock();

        if (shared.estop == 0) {
            pixel.setPixelColor(0, pixel.Color(0, 0, 0)); 
            pixel.show();
        }
    }
    if (cmd.e_stop) {
        assert_estop(ESTOP_RADIO_COMMAND);
    }

    // radio-layer watchdog
    if (millis() - lastGoodPacket > WATCHDOG_TIMEOUT) {
        assert_estop(ESTOP_RADIO_TIMEOUT);
        emergency_stop();
        lastGoodPacket = millis();
    }
    
    lock();
    shared.core0_last_update = millis(); // Badump
    unlock();

    bool now_estop;
    uint16_t reasons;

    lock();
    now_estop = shared.estop;
    reasons = shared.estop_flags;
    unlock();

    if (now_estop && !last_estop_state) {
        Serial.println("\n🚨 ESTOP ASSERTED — REASONS:"); //fuck the haters. Emojis in code is fun

        if (reasons & ESTOP_RADIO_COMMAND)   Serial.println(" • Radio command ESTOP");
        if (reasons & ESTOP_RADIO_TIMEOUT)   Serial.println(" • Radio timeout");
        if (reasons & ESTOP_CORE0_TIMEOUT)   Serial.println(" • Core0 heartbeat lost");
        if (reasons & ESTOP_REAR_DEVICE)     Serial.println(" • Rear axle fault");
        if (reasons & ESTOP_FRONT_DEVICE)    Serial.println(" • Front axle fault");
        if (reasons & ESTOP_STEERING_DEVICE) Serial.println(" • Steering fault");
        if (reasons & ESTOP_CHECKSUM_FAULT)  Serial.println(" • Radio checksum fault");

        Serial.println("------------------------------");
    }

    last_estop_state = now_estop;

}
