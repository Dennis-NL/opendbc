#include "safety_volkswagen_common.h"

// lateral limits
const SteeringLimits VOLKSWAGEN_MLB_STEERING_LIMITS = {
  .max_steer = 300,
  .max_rt_delta = 188,
  .max_rt_interval = 250000,
  .max_rate_up = 10,
  .max_rate_down = 10,
  .driver_torque_allowance = 60,
  .driver_torque_factor = 3,
  .type = TorqueDriverLimited,
};

// Transmit of LS_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
const CanMsg VOLKSWAGEN_MLB_STOCK_TX_MSGS[] = {
  {MSG_HCA_01, 0, 8},
  {MSG_LS_01, 0, 4},
  {MSG_LS_01, 2, 4},
  {MSG_LDW_02, 0, 8},
  {MSG_ACC_02, 0, 8}
};

RxCheck volkswagen_mlb_rx_checks[] = {
  {.msg = {{MSG_ESP_03, 0, 8, .check_checksum = false, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{MSG_LH_EPS_03, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{MSG_ESP_05, 0, 8, .check_checksum = false, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{MSG_TSK_02, 0, 8, .check_checksum = false, .max_counter = 15U, .frequency = 33U}, { 0 }, { 0 }}},
  {.msg = {{MSG_MOTOR_03, 0, 8, .check_checksum = false, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
};

static safety_config volkswagen_mlb_init(uint16_t param) {
  UNUSED(param);

  volkswagen_brake_pedal_switch = false;
  volkswagen_brake_pressure_detected = false;

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  return BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_STOCK_TX_MSGS);
}

static void volkswagen_mlb_rx_hook(const CANPacket_t *to_push) {
  if (GET_BUS(to_push) != 0U) return;

  int addr = GET_ADDR(to_push);

  if (addr == MSG_ESP_03) {
    uint32_t speed = 0;
    speed += ((GET_BYTE(to_push, 3) & 0xFU) << 8) | GET_BYTE(to_push, 2);  // FL
    speed += (GET_BYTE(to_push, 4) << 4) | (GET_BYTE(to_push, 3) >> 4);    // FR
    speed += ((GET_BYTE(to_push, 6) & 0xFU) << 8) | GET_BYTE(to_push, 5);  // RL
    speed += (GET_BYTE(to_push, 7) << 4) | (GET_BYTE(to_push, 6) >> 4);    // RR
    vehicle_moving = speed > 0U;
  }

  if (addr == MSG_LH_EPS_03) {
    update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(to_push));
  }

  if (addr == MSG_TSK_02) {
    int acc_status = (GET_BYTE(to_push, 2) & 0x3U);
    bool cruise_engaged = (acc_status == 1) || (acc_status == 2);
    acc_main_on = cruise_engaged || (acc_status == 0);
    pcm_cruise_check(cruise_engaged);
  }

  if (addr == MSG_LS_01) {
    if (GET_BIT(to_push, 13U) == 1U) controls_allowed = false;
  }

  if (addr == MSG_MOTOR_03) {
    gas_pressed = GET_BYTE(to_push, 6) != 0U;
    volkswagen_brake_pedal_switch = GET_BIT(to_push, 35U);
  }

  if (addr == MSG_ESP_05) {
    volkswagen_brake_pressure_detected = volkswagen_mlb_mqb_brake_pressure_threshold(to_push);
  }

  brake_pressed = volkswagen_brake_pedal_switch || volkswagen_brake_pressure_detected;

  generic_rx_checks((addr == MSG_HCA_01));
}

static bool volkswagen_mlb_tx_hook(const CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);
  bool tx = true;

  if (addr == MSG_HCA_01) {
    int desired_torque = GET_BYTE(to_send, 2) | ((GET_BYTE(to_send, 3) & 0x3FU) << 8);
    int sign = (GET_BYTE(to_send, 3) & 0x80U) >> 7;
    if (sign) desired_torque *= -1;

    if (steer_torque_cmd_checks(desired_torque, -1, VOLKSWAGEN_MLB_STEERING_LIMITS)) tx = false;
  }

  if ((addr == MSG_LS_01) && !controls_allowed) {
    if (GET_BIT(to_send, 16U) || GET_BIT(to_send, 19U)) tx = false;
  }

  return tx;
}

static int volkswagen_mlb_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      bus_fwd = 2;
      break;
    case 2:
      if ((addr == MSG_HCA_01) || (addr == MSG_LDW_02)) bus_fwd = -1;
      else if (volkswagen_longitudinal && ((addr == MSG_ACC_02) || (addr == MSG_ACC_06) || (addr == MSG_ACC_07))) bus_fwd = -1;
      else bus_fwd = 0;
      break;
    default:
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

const safety_hooks volkswagen_mlb_hooks = {
  .init = volkswagen_mlb_init,
  .rx = volkswagen_mlb_rx_hook,
  .tx = volkswagen_mlb_tx_hook,
  .fwd = volkswagen_mlb_fwd_hook,
  .get_counter = volkswagen_mqb_mlb_meb_get_counter,
  .get_checksum = volkswagen_mqb_mlb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_mlb_meb_compute_crc,
};

