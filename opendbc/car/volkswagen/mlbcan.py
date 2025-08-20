from opendbc.car.crc import CRC8H2F


def create_steering_control(packer, bus, apply_torque, lkas_enabled):
  values = {
    "HCA_01_Status_HCA": 7 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(int(apply_torque)),
    "HCA_01_LM_OffSign": 1 if apply_torque < 0 else 0,
    "HCA_01_Vib_Freq": 14,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control):
  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",
      "LDW_SW_Warnung_rechts",
      "LDW_Seite_DLCTLC",
      "LDW_DLC",
      "LDW_TLC",
    ]}
  values.update({
    "LDW_Status_LED_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if getattr(hud_control, "leftLaneDepart", False) else 1 + int(getattr(hud_control, "leftLaneVisible", False)),
    "LDW_Lernmodus_rechts": 3 if getattr(hud_control, "rightLaneDepart", False) else 1 + int(getattr(hud_control, "rightLaneVisible", False)),
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_acc_buttons_control(packer, bus, ls_stock_values, cancel=False, resume=False):
  values = {s: ls_stock_values[s] for s in [
    "LS_Hauptschalter",
    "LS_Typ_Hauptschalter",
    "LS_Codierung",
    "LS_Tip_Stufe_2",
  ]}
  values.update({
    "COUNTER": (ls_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": bool(cancel),
    "LS_Tip_Wiederaufnahme": bool(resume),
    "LS_Tip_Setzen": 0,
    "LS_Tip_Runter": 0,
    "LS_Tip_Hoch": 0,
  })
  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    return 6
  if long_active:
    return 3
  if main_switch_on:
    return 2
  return 0


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  return []


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": int(distance) + 2,
    "ACC_Display_Prio": 3,
    "ACC_Abstandsindex": int(lead_distance),
  }
  return packer.make_can_msg("ACC_02", bus, values)


def volkswagen_mqb_mlb_meb_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0xFF
  for i in range(1, len(d)):
    crc ^= d[i]
    crc = CRC8H2F[crc]
  counter = d[1] & 0x0F
  const = VOLKSWAGEN_MLB_CONSTANTS.get(address)
  if const:
    crc ^= const[counter]
    crc = CRC8H2F[crc]
  return crc ^ 0xFF


def xor_checksum(address: int, sig, d: bytearray) -> int:
  checksum = 0
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum ^= d[i]
  return checksum


VOLKSWAGEN_MLB_CONSTANTS: dict[int, list[int]] = {
  0x126: [0xDA] * 16, # HCA_01
  0x30C: [0x0F] * 16, # ACC_02
}
