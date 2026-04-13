#pragma once
#include <stdint.h>

struct __attribute__((packed)) axlepacket { //5 bytes + 1 byte address
    uint8_t device_type;    // 0x10 for axles, increment with number of axles
    uint8_t estop;          // 0 or 1
    uint8_t brake;          // 0 or 1
    uint8_t direction;      // 0 or 1
    uint8_t pwm;            // 0-255
};

struct __attribute__((packed)) steeringpacket { //4 bytes + 1 byte address
    uint8_t device_type;    //0x20 for steering
    uint8_t estop;          //0 or 1
    int16_t position;       //-512-511
};

struct __attribute__((packed)) lightpacket { //3 bytes + 1 byte address
    uint8_t device_type;    //0x30 for lights
    uint8_t estop;          //0 or 1
    uint8_t lights;         // 0 or 1, off or on
};