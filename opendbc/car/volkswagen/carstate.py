from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import DBC, CanBus, NetworkLocation, TransmissionType, GearShifter, \
                                                      CarControllerParams, VolkswagenFlags

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.frame = 0
    self.eps_init_complete = False
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}
    self.esp_hold_confirmation = False
    self.upscale_lead_car_signal = False
    self.eps_stock_values = False

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.CP.pcmCruise:
      for b in buttonEvents:
        # Enable OP long on falling edge of enable buttons
        if b.type in (ButtonType.setCruise, ButtonType.resumeCruise) and not b.pressed:
          return True
    return False

  def create_button_events(self, pt_cp, buttons):
    button_events = []

    for button in buttons:
      state = pt_cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]
    ext_cp = pt_cp if self.CP.networkLocation == NetworkLocation.fwdCamera else cam_cp

    if self.CP.flags & VolkswagenFlags.PQ:
      return self.update_pq(pt_cp, cam_cp, ext_cp)

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # -------------------------
    # MLB PLATFORM (0.9.8 port)
    # -------------------------
    if self.CP.flags & VolkswagenFlags.MLB:
      # Wielsnelheden (ESP_03)
      # ESP_[VL|VR|HL|HR]_Radgeschw zijn 12-bit, verdeeld over bytes; we reconstrueren zoals in 0.9.8
      fl = ((pt_cp.vl["ESP_03"]["ESP_VL_Radgeschw_H"] & 0xF) << 8) | pt_cp.vl["ESP_03"]["ESP_VL_Radgeschw_L"] if "ESP_VL_Radgeschw_H" in pt_cp.vl["ESP_03"] else pt_cp.vl["ESP_03"]["ESP_VL_Radgeschw"]
      fr = ((pt_cp.vl["ESP_03"]["ESP_VR_Radgeschw_H"] & 0xF) << 8) | pt_cp.vl["ESP_03"]["ESP_VR_Radgeschw_L"] if "ESP_VR_Radgeschw_H" in pt_cp.vl["ESP_03"] else pt_cp.vl["ESP_03"]["ESP_VR_Radgeschw"]
      rl = ((pt_cp.vl["ESP_03"]["ESP_HL_Radgeschw_H"] & 0xF) << 8) | pt_cp.vl["ESP_03"]["ESP_HL_Radgeschw_L"] if "ESP_HL_Radgeschw_H" in pt_cp.vl["ESP_03"] else pt_cp.vl["ESP_03"]["ESP_HL_Radgeschw"]
      rr = ((pt_cp.vl["ESP_03"]["ESP_HR_Radgeschw_H"] & 0xF) << 8) | pt_cp.vl["ESP_03"]["ESP_HR_Radgeschw_L"] if "ESP_HR_Radgeschw_H" in pt_cp.vl["ESP_03"] else pt_cp.vl["ESP_03"]["ESP_HR_Radgeschw"]
      self.parse_wheel_speeds(ret, fl, fr, rl, rr)

      # Gas/Rem uit Motor_03 + ESP_05
      ret.gasPressed = pt_cp.vl["Motor_03"]["MO_Fahrpedalrohwert_01"] > 0
      brake_pedal_pressed = bool(pt_cp.vl["Motor_03"]["MO_Fahrer_bremst"]) if "MO_Fahrer_bremst" in pt_cp.vl["Motor_03"] else False
      brake_pressure_detected = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
      ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0
      ret.brakePressed = brake_pedal_pressed or brake_pressure_detected

      # EPS / stuur / yaw (LH_EPS_03, LWI_01, ESP_02)
      ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
      ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
      ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
      ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
      hca_status = self.CCP.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
      ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)
      ret.yawRate = pt_cp.vl["ESP_02"]["ESP_Gierrate"] * (1, -1)[int(pt_cp.vl["ESP_02"]["ESP_VZ_Gierrate"])] * CV.DEG_TO_RAD

      # Blinkers (Blinkmodi_01)
      ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_01"]["BM_links"])
      ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_01"]["BM_rechts"])

      # ESP uitgeschakeld?
      ret.espDisabled = pt_cp.vl["ESP_01"]["ESP_Tastung_passiv"] != 0 if "ESP_01" in pt_cp.vl else False

      # Versnelling (MLB: geen shifter-enum in 0.9.8 — zet op Drive)
      ret.gearShifter = GearShifter.drive

      # ACC status via TSK_02 (identiek aan 0.9.8 mapping)
      if "TSK_02" in pt_cp.vl:
        acc_status = pt_cp.vl["TSK_02"]["TSK_Status"]
        ret.cruiseState.available = acc_status in (0, 1, 2)
        ret.cruiseState.enabled = acc_status in (1, 2)
        ret.accFaulted = (acc_status == 3)
      else:
        ret.cruiseState.available = False
        ret.cruiseState.enabled = False
        ret.accFaulted = False

      # Handrem + gordel (Kombi_01, Airbag_02)
      ret.parkingBrake = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"]) if "Kombi_01" in pt_cp.vl else False
      ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3 if "Airbag_02" in pt_cp.vl else False

      # LDW voor radar-doorsturen (alleen als camera op cam-bus zit)
      self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera and "LDW_02" in cam_cp.vl else {}

      # HCA (EA) status vanuit camera wanneer aanwezig
      self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT and "HCA_01" in cam_cp.vl:
        ret.carFaultedNonCritical = bool(cam_cp.vl["HCA_01"]["EA_Ruckfreigabe"]) or cam_cp.vl["HCA_01"]["EA_ACC_Sollstatus"] > 0

      # Cruise set-speed via ACC_02 als pcmCruise actief (zoals MQB)
      if self.CP.pcmCruise and "ACC_02" in ext_cp.vl:
        ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw_02"] * CV.KPH_TO_MS
        if ret.cruiseState.speed > 90:
          ret.cruiseState.speed = 0

      # Buttons (LS_01 knoppen pass-through) en events
      self.gra_stock_values = pt_cp.vl["LS_01"] if "LS_01" in pt_cp.vl else {}
      ret.buttonEvents = self.create_button_events(pt_cp, self.CCP.BUTTONS)

      # Cluster snelheid (optioneel; op MLB niet altijd beschikbaar — val terug op vEgoRaw)
      ret.vEgoCluster = pt_cp.vl["Kombi_01"]["KBI_angez_Geschw"] * CV.KPH_TO_MS if "Kombi_01" in pt_cp.vl else ret.vEgoRaw

      # Standstill via vEgoRaw, cruise standstill via ESP hold (niet aanwezig op MLB → False)
      ret.cruiseState.standstill = False

      # Low-speed alert hysterese (zoals 0910)
      ret.lowSpeedAlert = self.update_low_speed_alert(ret.vEgo)

      self.frame += 1
      return ret, ret_sp

    # -------------------------
    # MQB PLATFORM (0910 default)
    # -------------------------
    if self.CP.transmissionType == TransmissionType.direct:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Motor_EV_01"]["MO_Waehlpos"], None))
    elif self.CP.transmissionType == TransmissionType.manual:
      if bool(pt_cp.vl["Gateway_72"]["BCM1_Rueckfahrlicht_Schalter"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Gateway_73"]["GE_Fahrstufe"], None))

    # MQB-specific (ongewijzigd t.o.v. jouw 0910)
    if self.CP.flags & VolkswagenFlags.KOMBI_PRESENT:
      self.upscale_lead_car_signal = bool(pt_cp.vl["Kombi_03"]["KBI_Variante"])  # Analog vs digital instrument cluster

    self.parse_wheel_speeds(ret,
      pt_cp.vl["ESP_19"]["ESP_VL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_VR_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HR_Radgeschw_02"],
    )

    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
      ret.carFaultedNonCritical = bool(cam_cp.vl["HCA_01"]["EA_Ruckfreigabe"]) or cam_cp.vl["HCA_01"]["EA_ACC_Sollstatus"] > 0  # EA

    drive_mode = True
    ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0  # FIXME: pressure in Bar
    brake_pedal_pressed = bool(pt_cp.vl["Motor_14"]["MO_Fahrer_bremst"])
    brake_pressure_detected = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
    ret.brakePressed = brake_pedal_pressed or brake_pressure_detected
    ret.parkingBrake = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])  # FIXME: include EPB when available

    ret.doorOpen = any([pt_cp.vl["Gateway_72"]["ZV_FT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_BT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HBFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HD_offen"]])

    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
    ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

    self.acc_type = ext_cp.vl["ACC_06"]["ACC_Typ"]
    self.esp_hold_confirmation = bool(pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"])
    acc_limiter_mode = ext_cp.vl["ACC_02"]["ACC_Gesetzte_Zeitluecke"] == 0
    speed_limiter_mode = bool(pt_cp.vl["TSK_06"]["TSK_Limiter_ausgewaehlt"])

    ret.cruiseState.available = pt_cp.vl["TSK_06"]["TSK_Status"] in (2, 3, 4, 5)
    ret.cruiseState.enabled = pt_cp.vl["TSK_06"]["TSK_Status"] in (3, 4, 5)
    ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw_02"] * CV.KPH_TO_MS if self.CP.pcmCruise else 0
    ret.accFaulted = pt_cp.vl["TSK_06"]["TSK_Status"] in (6, 7)

    ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Left"])
    ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Right"])

    # Shared logic
    ret.vEgoCluster = pt_cp.vl["Kombi_01"]["KBI_angez_Geschw"] * CV.KPH_TO_MS

    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status, drive_mode)

    ret.gasPressed = pt_cp.vl["Motor_20"]["MO_Fahrpedalrohwert_01"] > 0
    ret.espActive = bool(pt_cp.vl["ESP_21"]["ESP_Eingriff"])
    ret.espDisabled = pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"] != 0
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    ret.standstill = ret.vEgoRaw == 0
    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation
    ret.cruiseState.nonAdaptive = acc_limiter_mode or speed_limiter_mode
    if ret.cruiseState.speed > 90:
      ret.cruiseState.speed = 0

    self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}
    self.gra_stock_values = pt_cp.vl["GRA_ACC_01"]

    ret.buttonEvents = self.create_button_events(pt_cp, self.CCP.BUTTONS)

    ret.lowSpeedAlert = self.update_low_speed_alert(ret.vEgo)

    self.frame += 1
    return ret, ret_sp

  def update_pq(self, pt_cp, cam_cp, ext_cp) -> tuple[structs.CarState, structs.CarStateSP]:
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # vEgo via Bremse_1
    ret.vEgoRaw = pt_cp.vl["Bremse_1"]["Geschwindigkeit_neu__Bremse_1_"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw == 0

    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]["LH3_BLW"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_BLWSign"])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit"] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit_S"])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]["LH3_LM"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_LMSign"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)

    ret.gasPressed = pt_cp.vl["Motor_3"]["Fahrpedal_Rohsignal"] > 0
    ret.brake = pt_cp.vl["Bremse_5"]["BR5_Bremsdruck"] / 250.0
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]["Bremslichtschalter"])
    ret.parkingBrake = bool(pt_cp.vl["Kombi_1"]["Bremsinfo"])

    if self.CP.transmissionType == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_1"]["Waehlhebelposition__Getriebe_1_"], None))
    elif self.CP.transmissionType == TransmissionType.manual:
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Rueckfahr"])
      ret.gearShifter = GearShifter.reverse if reverse_light else GearShifter.drive

    ret.doorOpen = any([pt_cp.vl["Gate_Komf_1"]["GK1_Fa_Tuerkont"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_BT_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HL_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HR_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HD_Hauptraste"]])

    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_re"])

    self.ldw_stock_values = cam_cp.vl["LDW_Status"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    ret.stockFcw = False
    ret.stockAeb = False

    self.acc_type = ext_cp.vl["ACC_System"]["ACS_Typ_ACC"]
    ret.cruiseState.available = bool(pt_cp.vl["Motor_5"]["GRA_Hauptschalter"])
    ret.cruiseState.enabled = pt_cp.vl["Motor_2"]["GRA_Status"] in (1, 2)
    if self.CP.pcmCruise:
      ret.accFaulted = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_StaACC"] in (6, 7)
    else:
      ret.accFaulted = pt_cp.vl["Motor_2"]["GRA_Status"] == 3

    ret.cruiseState.speed = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_V_Wunsch"] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:
      ret.cruiseState.speed = 0

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(300, pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_li"],
                                                                            pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_re"])
    ret.buttonEvents = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    self.gra_stock_values = pt_cp.vl["GRA_Neu"]

    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]["ESP_Passiv_getastet"])

    ret.lowSpeedAlert = self.update_low_speed_alert(ret.vEgo)

    self.frame += 1
    return ret, ret_sp

  def update_low_speed_alert(self, v_ego: float) -> bool:
    # Low speed steer alert hysteresis logic
    if (self.CP.minSteerSpeed - 1e-3) > CarControllerParams.DEFAULT_MIN_STEER_SPEED and v_ego < (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = True
    elif v_ego > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    return self.low_speed_alert

  def update_hca_state(self, hca_status, drive_mode=True):
    # Treat FAULT as temporary for worst likely EPS recovery time, for cars without factory Lane Assist
    # DISABLED means the EPS hasn't been configured to support Lane Assist
    self.eps_init_complete = self.eps_init_complete or (hca_status in ("DISABLED", "READY", "ACTIVE") or self.frame > 600)
    perm_fault = drive_mode and hca_status == "DISABLED" or (self.eps_init_complete and hca_status == "FAULT")
    temp_fault = drive_mode and hca_status in ("REJECTED", "PREEMPTED") or not self.eps_init_complete
    return temp_fault, perm_fault

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    if CP.flags & VolkswagenFlags.PQ:
      return CarState.get_can_parsers_pq(CP)

    cam_messages = []
    if CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
      cam_messages += [
        ("HCA_01", 1),  # From R242 Driver assistance camera, 50Hz if steering/1Hz if not
      ]

    # Voor MQB hebben we in 0.9.10 alleen de “trage” Blinkmodi_02 expliciet toegevoegd.
    # Voor MLB doen we hetzelfde voor Blinkmodi_01 en ACC_02 (zeldzaam/0 Hz op sommige auto's).
    pt_messages = []
    if CP.flags & VolkswagenFlags.MLB:
      pt_messages += [
        ("Blinkmodi_01", 1),
        ("ACC_02", 0),
      ]
    else:
      pt_messages += [
        ("Blinkmodi_02", 1),
      ]

    # Camera-bus LDW afhankelijk van locatie
    if CP.networkLocation == NetworkLocation.fwdCamera:
      cam_messages += [("LDW_02", 10)]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).cam),
    }

  @staticmethod
  def get_can_parsers_pq(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).cam),
    }
