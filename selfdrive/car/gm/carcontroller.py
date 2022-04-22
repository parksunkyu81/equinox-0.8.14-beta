from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from common.conversions import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, NO_ASCM, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.controls.lib.drive_helpers import V_CRUISE_ENABLE_MIN

VisualAlert = car.CarControl.HUDControl.VisualAlert
GearShifter = car.CarState.GearShifter

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.comma_pedal = 0.

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False

    self.params = CarControllerParams(CP)

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

  def update(self, c, enabled, CS, frame, actuators,
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

      if CS.CP.enableGasInterceptor:
        # 이것이 없으면 저속에서 너무 공격적입니다.
        acc_mult = interp(CS.out.vEgo, [0., 5.], [0.17, 0.24])
        if c.active and CS.adaptive_Cruise and CS.out.vEgo > V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH:
          self.comma_pedal = clip(actuators.accel * acc_mult, 0., 1.)
          actuators.commaPedal = self.comma_pedal  # for debug value
        elif not c.active or not CS.adaptive_Cruise or CS.out.vEgo <= V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH:
          self.comma_pedal = 0

          ###발차시점, 처음 0~6초까지 , 1.5-6초간만 페달에 0.1775를 더한다.

          d = 0
          lead = self.scc_smoother.get_lead(controls.sm)
          if lead is not None:
            d = lead.dRel

          frameDivider = 25  # , 상태변화 해상도를 염려하여, 틱당 0.25초 기준으로한다.
          stoppingStateWindowsActiveCounterLimitstoMS = 1250
          if not self.stoppingStateWindowsActive:
            actuators.pedalStartingAdder = 0
            actuators.pedalDistanceAdder = 0
            if (self.frame % frameDivider) == 0:
              self.beforeStoppingState = self.currentStoppingState
              self.currentStoppingState = (controls.LoC.long_control_state == LongCtrlState.stopping)

          if self.beforeStoppingState and not self.currentStoppingState and not self.stoppingStateWindowsActive:
            self.stoppingStateWindowsActive = True

          if self.stoppingStateWindowsActive:
            self.stoppingStateWindowsActiveCounter += 1
            if self.stoppingStateWindowsActiveCounter > (0):
              actuators.pedalStartingAdder = interp(CS.out.vEgo, [0.0, 9 * CV.KPH_TO_MS, 18.0 * CV.KPH_TO_MS],
                                                    [0.1875, 0.2075, 0.0050])
              if d > 0:
                actuators.pedalDistanceAdder = interp(d, [1, 10, 25, 40], [-0.0250, -0.0075, 0.0075, 0.0550])

            if self.stoppingStateWindowsActiveCounter > (stoppingStateWindowsActiveCounterLimitstoMS) or (
                    controls.LoC.long_control_state == LongCtrlState.stopping) or CS.out.vEgo > 30 * CV.KPH_TO_MS:
              self.stoppingStateWindowsActive = False
              self.stoppingStateWindowsActiveCounter = 0
              self.beforeStoppingState = False
              self.currentStoppingState = False
              actuators.pedalStartingAdder = 0
              actuators.pedalDistanceAdder = 0

            actuators.pedalAdderFinal = (actuators.pedalStartingAdder + actuators.pedalDistanceAdder)
            self.comma_pedal += interp(self.stoppingStateWindowsActiveCounter,
                                       [0, stoppingStateWindowsActiveCounterLimitstoMS], [actuators.pedalAdderFinal, 0])

            # 발차 .. ? frame 1 당 0.01초

          actuators.commaPedal = self.comma_pedal  # for debug value

          if actuators.accel < 0.105:
            can_sends.append(gmcan.create_regen_paddle_command(self.packer_pt, CanBus.POWERTRAIN))
            actuators.regenPaddle = True  # for icon


          elif controls.LoC.pid.f < - 0.625:
            can_sends.append(gmcan.create_regen_paddle_command(self.packer_pt, CanBus.POWERTRAIN))
            actuators.regenPaddle = True  # for icon
        else:
          self.comma_pedal = 0.0  # Must be set by zero, otherwise cannot re-acceling when stopped. - jc01rho.

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
    #new_actuators.gas = self.comma_pedal

    return new_actuators, can_sends
