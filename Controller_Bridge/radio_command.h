#pragma once
#include <stdint.h>

struct __attribute__((packed)) radio_command {
    uint8_t magic;          // 0x4B
    uint8_t e_stop;         // 0 or 1
    int16_t throttle;       // 0-1023 (actual Xbox trigger value)
    int16_t brake_trigger;  // 0-1023 (actual Xbox trigger value)
    uint8_t brake_button;   // 0 or 1
    uint8_t front_brake; // 0 or 1, does what it says
    int16_t steering;       // -512 to +511 (actual Xbox stick value)
    uint8_t lights;         // 0 or 1
    uint8_t checksum;       // XOR of all bytes
};