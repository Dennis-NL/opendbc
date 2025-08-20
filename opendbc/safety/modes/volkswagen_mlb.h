#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

static safety_config volkswagen_mlb_init(uint16_t param) {
  UNUSED(param);

  // TX: LS_01 ook op bus 2 ivm gateway/camera
  static const CanMsg VOLKSWAGEN_MLB_STOCK_TX_MSGS[] = {
    {MSG_HCA_01, 0, 8, .check_relay = true},
    {MSG_LS_01,  0, 4, .check_relay = false},
    {MSG_LS_01,  2, 4, .check_relay = false},
    {MSG_LDW_02, 0, 8, .check_relay = true},
    {MSG_ACC_02, 0, 8, .check_relay = true},
  };

  // RX: 0.9.10 API: freq als 4e arg, ignore_* flags
  static RxCheck volkswagen_mlb_rx_checks[] = {
    {.msg = {{MSG_ESP_03,    0, 8,  50U, .max_counter = 15U, .ignore_checksum = true,  .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, 100U, .max_counter = 15U,                           .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESP_05,    0, 8,  50U, .max_counter = 15U, .ignore_checksum = true,  .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_TSK_02,    0, 8,  33U, .max_counter = 15U, .ignore_checksum = true,  .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_03,  0, 8, 100U, .max_counter = 15U, .ignore_checksum = true,  .ignore_quality_flag = true}, { 0 }, { 0 }}},
    // Sommige MLB’s gebruiken TSK_06 ipv TSK_02
    {.msg = {{MSG_TSK_06,    0, 8,  50U, .max_counter = 15U, .ignore_checksum = true,  .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  volkswagen_brake_pedal_switch = false;
  volkswagen_brake_pressure_detected = false;

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  return BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_STOCK_TX_MSGS);
}

static void volkswagen_mlb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus != 0U) return;

  const int addr = msg->addr;

  if (addr == MSG_ESP_03) {
    uint32_t speed = 0U;
    speed += ((msg->data[3] & 0x0FU) << 8) | msg->data[2];
    speed += (msg->data[4] << 4) | (msg->data[3] >> 4);
    speed += ((msg->data[6] & 0x0FU) << 8) | msg->data[5];
    speed += (msg->data[7] << 4) | (msg->data[6] >> 4);
    vehicle_moving = (speed > 0U);
  }

  if (addr == MSG_LH_EPS_03) {
    update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
  }

  if ((addr == MSG_TSK_02) || (addr == MSG_TSK_06)) {
    const int acc_status = (msg->data[2] & 0x3U);
    const bool cruise_engaged = (acc_status == 1) || (acc_status == 2);
    acc_main_on = cruise_engaged || (acc_status == 0);
    pcm_cruise_check(cruise_engaged);
  }

  if (addr == MSG_LS_01) {
    if (GET_BIT(msg, 13U)) {
      controls_allowed = false;
    }
  }

  if (addr == MSG_MOTOR_03) {
    gas_pressed = (msg->data[6] != 0U);
    volkswagen_brake_pedal_switch = GET_BIT(msg, 35U);
  }

  if (addr == MSG_ESP_05) {
    volkswagen_brake_pressure_detected = volkswagen_mlb_mqb_brake_pressure_threshold(msg);
  }

  brake_pressed = volkswagen_brake_pedal_switch || volkswagen_brake_pressure_detected;
}

static bool volkswagen_mlb_tx_hook(const CANPacket_t *msg) {
  const int addr = msg->addr;
  bool tx = true;

  if (addr == MSG_HCA_01) {
    const TorqueSteeringLimits VOLKSWAGEN_MLB_STEERING_LIMITS = {
      .max_torque = 300,
      .max_rt_delta = 188,
      .max_rate_up = 10,
      .max_rate_down = 10,
      .driver_torque_allowance = 60,
      .driver_torque_multiplier = 3,
      .type = TorqueDriverLimited,
    };

    int desired_torque = msg->data[2] | ((msg->data[3] & 0x3FU) << 8);
    const int sign = (msg->data[3] & 0x80U) >> 7;
    if (sign) desired_torque *= -1;

    if (steer_torque_cmd_checks(desired_torque, -1, VOLKSWAGEN_MLB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  if ((addr == MSG_LS_01) && !controls_allowed) {
    if (GET_BIT(msg, 16U) || GET_BIT(msg, 19U)) {
      tx = false;
    }
  }

  return tx;
}

const safety_hooks volkswagen_mlb_hooks = {
  .init = volkswagen_mlb_init,
  .rx = volkswagen_mlb_rx_hook,
  .tx = volkswagen_mlb_tx_hook,
  .get_counter = volkswagen_mqb_mlb_meb_get_counter,
  .get_checksum = volkswagen_mqb_mlb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_mlb_meb_compute_crc,
};
