from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from common.conversions import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, NO_ASCM, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.controls.lib.drive_helpers import V_CRUISE_ENABLE_MIN
from selfdrive.ntune import ntune_scc_get

VisualAlert = car.CarControl.HUDControl.VisualAlert
GearShifter = car.CarState.GearShifter
LongCtrlState = car.CarControl.Actuators.LongControlState

CREEP_SPEED = 1.12   # 4km

class CarController():

  def get_lead(self, sm):
    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne
    return None

  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.comma_pedal = 0.0
    self.accel = 0
    self.pedalMaxValue = 0.3310

    self.currentStoppingState = False
    self.beforeStoppingState = False
    self.stoppingStateTimeWindowsActive = False
    self.stoppingStateTimeWindowsActiveCounter = 0
    self.stoppingStateTimeWindowsClosingAdder = 0
    self.stoppingStateTimeWindowsClosing = False
    self.stoppingStateTimeWindowsClosingCounter = 0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False

    self.params = CarControllerParams(CP)

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

  def update(self, c, enabled, CS, frame, controls, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # Send CAN commands.
    can_sends = []

    # Steering (50Hz)
    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (frame % P.STEER_STEP) == 0:
      lkas_enabled = c.active and not (CS.out.steerFaultTemporary or CS.out.steerFaultPermanent) and CS.out.vEgo > P.MIN_STEER_SPEED
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      self.apply_steer_last = apply_steer
      # GM EPS faults on any gap in received message counters. To handle transient OP/Panda safety sync issues at the
      # moment of disengaging, increment the counter based on the last message known to pass Panda safety checks.
      idx = (CS.lka_steering_cmd_counter + 1) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

      self.accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      #forward_distance = 0
      if CS.CP.enableGasInterceptor:
        # 이것이 없으면 저속에서 너무 공격적입니다.
        if c.active and CS.adaptive_Cruise and CS.out.vEgo > V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH:

          ## ================================================================================== ##

          accelFomula = (actuators.accel / 8.8 if actuators.accel >= 0 else actuators.accel / 9.25)
          accelFomula = round(accelFomula, 3)
          pedalValue = interp(CS.out.vEgo, [0., 18.0 * CV.KPH_TO_MS], [0.1625, 0.2125]) + accelFomula
          pedalValue = min(pedalValue, interp(CS.out.vEgo, [0., 19.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS],
                                              [0.2550, 0.2750, 0.3150]))

          self.comma_pedal_original = pedalValue  # (actuators.accel * acc_mult, 0., 1.)
          self.comma_pedal_new = clip(
            interp(actuators.accel, [-0.85, -0.325, 0.00, 0.20], [0.0, 0.1600, 0.2190, 0.22150]) + accelFomula, 0., 1.)

          gapInterP = interp(CS.out.vEgo, [19 * CV.KPH_TO_MS, 45 * CV.KPH_TO_MS], [1, 0])
          self.comma_pedal = (gapInterP * self.comma_pedal_original) + ((1.0 - gapInterP) * self.comma_pedal_new)

          self.comma_pedal = clip(self.comma_pedal, 0.0,
                                  interp(actuators.accel, [0.85, 1.5], [0.0000, 0.0200]) + self.pedalMaxValue)  # 급가속 방

          if CS.CP.restartForceAccel:
            d = 0
            lead = self.get_lead(controls.sm)
            if lead is not None:
              d = lead.dRel

            stoppingStateWindowsActiveCounterLimits = 1500  # per 0.01s,
            if not self.stoppingStateTimeWindowsActive:
              actuators.pedalStartingAdder = 0
              actuators.pedalDistanceAdder = 0
              self.beforeStoppingState = self.currentStoppingState
              self.currentStoppingState = (controls.LoC.long_control_state == LongCtrlState.stopping)

            if self.beforeStoppingState and not self.currentStoppingState and not self.stoppingStateTimeWindowsActive:
              self.stoppingStateTimeWindowsActive = True

            if self.stoppingStateTimeWindowsActive:
              if not self.stoppingStateTimeWindowsClosing:
                self.stoppingStateTimeWindowsActiveCounter += 1
                actuators.stoppingStateTimeWindowsActiveCounter = self.stoppingStateTimeWindowsActiveCounter
                if self.stoppingStateTimeWindowsActiveCounter > 0:
                  actuators.pedalStartingAdder = interp(CS.out.vEgo, [0.0, 5.0 * CV.KPH_TO_MS, 12.5 * CV.KPH_TO_MS,
                                                                      25.0 * CV.KPH_TO_MS],
                                                        [0.1850, 0.2275, 0.1750, 0.025])
                  if d > 0:
                    actuators.pedalDistanceAdder = interp(d, [1, 6, 8, 9.5, 15, 30],
                                                          [-1.0250, -0.5000, -0.0525, -0.0100, 0.0175, 0.1000])
                  actuators.pedalAdderFinal = (actuators.pedalStartingAdder + actuators.pedalDistanceAdder)

                if self.stoppingStateTimeWindowsActiveCounter > (stoppingStateWindowsActiveCounterLimits) \
                        or (controls.LoC.long_control_state == LongCtrlState.stopping) \
                        or CS.out.vEgo > 5 * CV.KPH_TO_MS \
                        or controls.LoC.pid.f < -0.65 \
                        or actuators.accel < - 1.15:
                  if controls.LoC.pid.f < -0.625 or actuators.accel < - 1.225:
                    self.stoppingStateTimeWindowsClosingAdder = 0
                  else:
                    self.stoppingStateTimeWindowsClosingAdder = actuators.pedalAdderFinal
                  self.stoppingStateTimeWindowsActiveCounter = 0
                  self.beforeStoppingState = False
                  self.currentStoppingState = False
                  actuators.pedalStartingAdder = 0
                  actuators.pedalDistanceAdder = 0
                  actuators.pedalAdderFinal = 0
                  self.stoppingStateTimeWindowsClosing = True

              else:  # if self.stoppingStateTimeWindowsClosing :
                self.stoppingStateTimeWindowsClosingCounter += 1
                actuators.stoppingStateTimeWindowsClosingCounter = self.stoppingStateTimeWindowsClosingCounter
                actuators.pedalAdderFinal = interp(self.stoppingStateTimeWindowsClosingCounter,
                                                   [0, (stoppingStateWindowsActiveCounterLimits / 3)],
                                                   [self.stoppingStateTimeWindowsClosingAdder, 0])

                if self.stoppingStateTimeWindowsClosingAdder == 0 or (
                        self.stoppingStateTimeWindowsClosingCounter > (stoppingStateWindowsActiveCounterLimits / 3)):
                  self.stoppingStateTimeWindowsClosing = False
                  self.stoppingStateTimeWindowsClosingCounter = 0
                  self.stoppingStateTimeWindowsClosingAdder = 0
                  self.stoppingStateTimeWindowsActive = False

              self.comma_pedal += actuators.pedalAdderFinal
              self.comma_pedal = clip(self.comma_pedal, 0.0, (self.pedalMaxValue - 0.025))

          # braking logic
          """if actuators.accel < -0.15:
            actuators.regenPaddle = True  # for icon
          elif controls.LoC.pid.f < - 0.55:
            actuators.regenPaddle = True  # for icon
            minMultipiler = interp(CS.out.vEgo,
                                    [20 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 120 * CV.KPH_TO_MS],
                                    [0.850, 0.750, 0.625, 0.150])
            self.comma_pedal *= interp(controls.LoC.pid.f, [-2.25, -2.0, -1.5, -0.600],
                                        [0, 0.020, minMultipiler, 0.975])"""
          actuators.commaPedal = self.comma_pedal
          ## ================================================================================== ##

          """PEDAL_SCALE = interp(CS.out.vEgo, [0., 18.0 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 40 * CV.KPH_TO_MS],
                                            [0.22, 0.25, 0.27, 0.24])

          lead = self.get_lead(controls.sm)
          if lead is not None:
            forward_distance = lead.dRel
          else:
            forward_distance = 0

          if forward_distance > 0:
            start_boost = interp(CS.out.vEgo, [0.0, CREEP_SPEED, 2 * CREEP_SPEED], [0.19, 0.19, 0.0])
            is_accelerating = interp(actuators.accel, [0.0, 0.2], [0.0, 1.0])  # DEF : 1.0
            boost = start_boost * is_accelerating
            pedal_command = PEDAL_SCALE * (actuators.accel + boost)
          else:
            pedal_command = PEDAL_SCALE * actuators.accel
          ## ================================================ ##
          pedal_command = PEDAL_SCALE * actuators.accel
          self.comma_pedal = clip(pedal_command, 0., 1.)"""

        elif not c.active or not CS.adaptive_Cruise or CS.out.vEgo <= V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH:
          self.comma_pedal = 0.0
          actuators.commaPedal = self.comma_pedal
        if (frame % 4) == 0:
          idx = (frame // 4) % 4
          can_sends.append(create_gas_interceptor_command(self.packer_pt, self.comma_pedal, idx))

    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
      steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX
    new_actuators.accel = self.accel
    new_actuators.gas = self.comma_pedal

    return new_actuators, can_sends
