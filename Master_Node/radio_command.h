#pragma once
#include <stdint.h>

struct __attribute__((packed)) radio_command {
    uint8_t magic;       // 0x4B - sync/start marker - 0x4B is K... for kei truck
    uint8_t e_stop;      // 0 or 1
    int16_t throttle;    // 0 to 1023
    int16_t brake_trigger; // 0 to 1023
    uint8_t brake_button; // 0 or 1, functionally acts as a handbrake - engaging the speed controller brake and reducing pwm to 0 but only on the rear wheels.
    uint8_t front_brake; // 0 or 1, does what it say
    int16_t steering;    // -512 to 511
    uint8_t lights;      // 0 or 1
    uint8_t checksum;    // XOR of bytes 1-7 (not including magic)
};

