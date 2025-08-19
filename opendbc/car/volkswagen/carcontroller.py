import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    # --- Platform-selectie (inclusief MLB) ---
    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    # --- MLB/MQB HCA/EPS state ---
    self.apply_torque_last = 0
    self.frame = 0

    # Timer-mitigatie zoals 0.9.8 (geporteerd)
    self.eps_timer_workaround = True           # vervang desgewenst op fingerprint-basis
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0           # totale duur HCA actief
    self.hca_frame_timer_resetting = 0         # duur HCA geforceerd uit (voor reset)
    self.hca_frame_low_torque = 0              # duur met lage torque
    self.hca_frame_same_torque = 0             # duur met identieke torque

    # HUD/ACC helpers die al bestonden
    self.gra_acc_counter_last = None

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #
    if self.frame % self.CCP.STEER_STEP == 0:
      # 0.9.8-achtige mitigatie van EPS "uninterrupted steering" timer:
      # - Houd timer bij zolang we niet nul sturen
      # - Forceer af en toe 0 Nm bij lage torque of wanneer reset nodig is
      # - Vermijd te lang exact dezelfde torque te sturen

      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

        # Timer loopt wanneer we echt sturen (≠ 0)
        hca_enabled = abs(apply_torque) > 0
        if hca_enabled:
          self.hca_frame_timer_running += self.CCP.STEER_STEP

          # Detecteer "same torque too long" en nudge ±1
          if self.apply_torque_last == apply_torque:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_torque -= (1 if apply_torque > 0 else -1) if apply_torque != 0 else 1
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0

          # Low-torque venster om tijdig te resetten (zoals 0.9.8)
          if self.eps_timer_workaround and self.hca_frame_timer_running >= self.CCP.STEER_TIME_BM / DT_CTRL:
            if abs(apply_torque) <= self.CCP.STEER_LOW_TORQUE:
              self.hca_frame_low_torque += self.CCP.STEER_STEP
              if self.hca_frame_low_torque >= self.CCP.STEER_TIME_LOW_TORQUE / DT_CTRL:
                # Korte onderbreking: 1 frame 0 Nm sturen en hca_enabled uit zetten
                hca_enabled = False
                apply_torque = 0
                self.hca_frame_low_torque = 0
                self.hca_frame_timer_resetting = 0  # start reset-interval tellen
            else:
              self.hca_frame_low_torque = 0

        else:
          # Lat actief, maar torque clampte naar 0 -> resetcyclus
          self.hca_frame_low_torque = 0
          self.hca_frame_same_torque = 0
          self.hca_frame_timer_resetting += self.CCP.STEER_STEP
          # Indien we echt een resetpuls hebben gegeven, zet totale timer terug
          if self.hca_frame_timer_resetting >= self.CCP.STEER_TIME_RESET / DT_CTRL or not self.eps_timer_workaround:
            self.hca_frame_timer_running = 0

      else:
        # Lateral uit: alle timers terug en torque 0
        hca_enabled = False
        apply_torque = 0
        self.hca_frame_low_torque = 0
        self.hca_frame_same_torque = 0
        self.hca_frame_timer_resetting = 0
        self.hca_frame_timer_running = 0

      # Alert net als 0.9.8 wanneer we te dicht bij EPS tijdslimiet komen
      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_torque_last = apply_torque

      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Emergency Assist pacifier (2x OP torque of echte driver torque)
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #
    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation))

      # AEB placeholders (uitgezet, zoals in jouw 0.9.10)
      # if self.aeb_available:
      #   if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #     can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #   if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #     can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #
    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #
    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    # **** Actuator feedback ************************************************ #
    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
