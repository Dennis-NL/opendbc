#pragma once

#include <stdint.h>
#include <stdbool.h>

extern const uint16_t FLAG_VOLKSWAGEN_LONG_CONTROL;
const uint16_t FLAG_VOLKSWAGEN_LONG_CONTROL = 1;

static uint8_t volkswagen_crc8_lut_8h2f[256]; // CRC8 lookup table for poly 0x2F, 8H2F/AUTOSAR

extern bool volkswagen_longitudinal;
bool volkswagen_longitudinal = false;

extern bool volkswagen_set_button_prev;
bool volkswagen_set_button_prev = false;

extern bool volkswagen_resume_button_prev;
bool volkswagen_resume_button_prev = false;

extern bool volkswagen_brake_pedal_switch;
bool volkswagen_brake_pedal_switch = false;

extern bool volkswagen_brake_pressure_detected;
bool volkswagen_brake_pressure_detected = false;

// CAN message definitions
#define MSG_ACC_02      0x30CU
#define MSG_ACC_06      0x122U
#define MSG_ACC_07      0x12EU
#define MSG_ESP_03      0x103U
#define MSG_ESP_05      0x106U
#define MSG_ESP_19      0x0B2U
#define MSG_HCA_01      0x126U
#define MSG_LDW_02      0x397U
#define MSG_LH_EPS_03   0x09FU
#define MSG_LS_01       0x10BU
#define MSG_MOTOR_03    0x105U
#define MSG_MOTOR_14    0x3BEU
#define MSG_MOTOR_20    0x121U
#define MSG_TSK_02      0x10CU
#define MSG_TSK_06      0x120U
#define MSG_GRA_ACC_01  0x12BU

// ---------------------------
// Checksum and counter helpers
// ---------------------------

static uint32_t volkswagen_mqb_mlb_meb_get_checksum(const CANPacket_t *msg) {
    return (uint8_t)msg->data[0];
}

static uint8_t volkswagen_mqb_mlb_meb_get_counter(const CANPacket_t *msg) {
    return (uint8_t)msg->data[1] & 0xFU;
}

// ---------------------------
// CRC computation for MLB/MQB/MEB
// ---------------------------

static uint32_t volkswagen_mqb_mlb_meb_compute_crc(const CANPacket_t *msg) {
    int len = GET_LEN(msg);
    uint8_t crc = 0xFFU;
    for (int i = 1; i < len; i++) {
        crc ^= (uint8_t)msg->data[i];
        crc = volkswagen_crc8_lut_8h2f[crc];
    }

    uint8_t counter = volkswagen_mqb_mlb_meb_get_counter(msg);

    if (msg->addr == MSG_LH_EPS_03) {
        crc ^= (uint8_t[]){0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5}[counter];
    } else if (msg->addr == MSG_ESP_05) {
        crc ^= (uint8_t[]){0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07}[counter];
    } else if (msg->addr == MSG_TSK_06) {
        crc ^= (uint8_t[]){0xC4,0xE2,0x4F,0xE4,0xF8,0x2F,0x56,0x81,0x9F,0xE5,0x83,0x44,0x05,0x3F,0x97,0xDF}[counter];
    } else if (msg->addr == MSG_MOTOR_20) {
        crc ^= (uint8_t[]){0xE9,0x65,0xAE,0x6B,0x7B,0x35,0xE5,0x5F,0x4E,0xC7,0x86,0xA2,0xBB,0xDD,0xEB,0xB4}[counter];
    } else if (msg->addr == MSG_GRA_ACC_01) {
        crc ^= (uint8_t[]){0x6A,0x38,0xB4,0x27,0x22,0xEF,0xE1,0xBB,0xF8,0x80,0x84,0x49,0xC7,0x9E,0x1E,0x2B}[counter];
    }

    crc = volkswagen_crc8_lut_8h2f[crc];
    return (uint8_t)(crc ^ 0xFFU);
}

// ---------------------------
// Driver input helpers (MLB/MQB)
// ---------------------------

static int volkswagen_mlb_mqb_driver_input_torque(const CANPacket_t *msg) {
    int torque_driver_new = msg->data[5] | ((msg->data[6] & 0x1FU) << 8);
    bool sign = GET_BIT(msg, 55U);
    if (sign) torque_driver_new *= -1;
    return torque_driver_new;
}

static bool volkswagen_mlb_mqb_brake_pressure_threshold(const CANPacket_t *msg) {
    return GET_BIT(msg, 26U);
}
