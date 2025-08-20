# opendbc/car/volkswagen/mlbcan.py
from opendbc.car.crc import CRC8H2F

def create_steering_control(packer, bus, apply_torque, lkas_enabled):
  """
  MLB: HCA_01. Status en veldnamen volgens MLB DBC.
  Let op: MLB gebruikt doorgaans Status_HCA = 7 voor actief (MQB gebruikt 5).
  """
  values = {
    "HCA_01_Status_HCA": 7 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(int(apply_torque)),
    "HCA_01_LM_OffSign": 1 if apply_torque < 0 else 0,
    "HCA_01_Vib_Freq": 14,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    # Sommige MLB-varianten hebben dit veld; als het niet in je DBC zit, haal het weg:
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control):
  """
  MLB: LDW_02 HUD. Houdt stock-velden in stand en zet de LED/status bits.
  """
  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      # Deze bestaan ook op MLB varianten (namen uit je DBC):
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
  """
  MLB gebruikt LS_01 i.p.v. MQB's GRA_ACC_01 voor knop-events.
  Signaturen gelijk gehouden aan MQB (cancel/resume) voor compatibiliteit.
  """
  values = {s: ls_stock_values[s] for s in [
    "LS_Hauptschalter",           # ACC main on/off
    "LS_Typ_Hauptschalter",       # stalk type
    "LS_Codierung",               # coding
    "LS_Tip_Stufe_2",             # stalk subtype
  ]}

  values.update({
    "COUNTER": (ls_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": bool(cancel),
    "LS_Tip_Wiederaufnahme": bool(resume),
    # Overige tipjes uit (we sturen alleen resume/cancel in controls-off situaties volgens safety)
    "LS_Tip_Setzen": 0,
    "LS_Tip_Runter": 0,
    "LS_Tip_Hoch": 0,
  })
  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  # Zelfde mapping als MQB; pas aan als MLB afwijkende encodings vereist
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0
  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # Voor nu gelijk aan control; uitbreiden als MLB andere HUD states wil
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  """
  MLB stuurt (vooralsnog) géén ACC_06/ACC_07 (niet in safety whitelist).
  Geef daarom geen CAN-frames terug.
  """
  return []


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  """
  MLB: ACC_02 HUD. Velden gelijk aan MQB zolang je MLB DBC ze heeft.
  """
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": int(distance) + 2,
    "ACC_Display_Prio": 3,
    "ACC_Abstandsindex": int(lead_distance),
  }
  return packer.make_can_msg("ACC_02", bus, values)


# (optioneel) AEB ondersteunings-API — alleen definiëren als je ze elders aanroept.
# Als je safety ACC_10/15 niet toestaat, worden ze toch geblokkeerd door panda safety.
def create_aeb_control(packer, fcw_active, aeb_active, accel):
  values = {
    "AWV_Vorstufe": 0,
    "AWV1_Anf_Prefill": 0,
    "AWV1_HBA_Param": 0,
    "AWV2_Freigabe": 0,
    "AWV2_Ruckprofil": 0,
    "AWV2_Priowarnung": 0,
    "ANB_Notfallblinken": 0,
    "ANB_Teilbremsung_Freigabe": 0,
    "ANB_Zielbremsung_Freigabe": 0,
    "ANB_Zielbrems_Teilbrems_Verz_Anf": 0.0,
    "AWV_Halten": 0,
    "PCF_Time_to_collision": 0xFF,
  }
  return packer.make_can_msg("ACC_10", 0, values)


def create_aeb_hud(packer, aeb_supported, fcw_active):
  values = {
    "AWV_Texte": 5 if aeb_supported else 7,
    "AWV_Status_Anzeige": 1 if aeb_supported else 2,
  }
  return packer.make_can_msg("ACC_15", 0, values)


# ---- Checksums/consts (gedeeld algoritme met MQB/MLB/MEb) ----

def volkswagen_mqb_mlb_meb_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0xFF
  for i in range(1, len(d)):
    crc ^= d[i]
    crc = CRC8H2F[crc]
  counter = d[1] & 0x0F
  const = VOLKSWAGEN_MQB_MLB_MEB_CONSTANTS.get(address)
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


# Let op: laat alleen de IDs staan die in jouw MLB-DKC gebruikt worden.
# Verwijder gerust entries die MLB niet heeft; deze tabel beïnvloedt alleen CRC-selectie.
VOLKSWAGEN_MQB_MLB_MEB_CONSTANTS: dict[int, list[int]] = {
  0x126: [0xDA] * 16,  # HCA_01
  0x30C: [0x0F] * 16,  # ACC_02
  # Voeg hier MLB-specifieke IDs toe als je DBC ze vereist voor CRC/COUNTER combinaties
}
