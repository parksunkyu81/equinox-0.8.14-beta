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

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.comma_pedal = 0.
    self.accel = 0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False

    self.params = CarControllerParams(CP)

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

    self.longcontrol = CP.openpilotLongitudinalControl
    self.packer = CANPacker(dbc_name)
    self.comma_pedal = 0.
    self.comma_pedal_original = 0.
    self.currentStoppingState = False
    self.beforeStoppingState = False
    self.stoppingStateTimeWindowsActive = False
    self.stoppingStateTimeWindowsActiveCounter = 0
    self.stoppingStateTimeWindowsClosingAdder = 0
    self.stoppingStateTimeWindowsClosing = False
    self.stoppingStateTimeWindowsClosingCounter = 0
    self.pedalAdderClosing = 0


  def get_lead(self, sm):
    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne

    return None

  def update(self, c, enabled, CS, frame, actuators, controls,
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

      if CS.CP.enableGasInterceptor:
        if c.active and CS.adaptive_Cruise and CS.out.vEgo > V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH:

          """MAX_INTERCEPTOR_GAS = ntune_scc_get("sccGasFactor")  # default : 0.5
          PEDAL_SCALE = interp(CS.out.vEgo, [0.0, 19., 29.], [0.15, 0.3, 0.0])
          pedal_offset = interp(CS.out.vEgo, [0.0, 8 * CV.KPH_TO_MS, 68. * CV.KPH_TO_MS], [-.4, 0.0, 0.2])
          pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
          self.comma_pedal = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)"""

          acc_mult = interp(CS.out.vEgo, [0., 18.0 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 40 * CV.KPH_TO_MS],
                            [0.17, 0.24, 0.265, 0.24])
          self.comma_pedal_original = clip(actuators.accel * acc_mult, 0., 1.)
          self.comma_pedal_new = clip(interp(actuators.accel, [-1.0, 0.0, 0.2], [0.0, 0.22, 0.222])
                                      + (actuators.accel / 10), 0., 1.)

          gapInterP = interp(CS.out.vEgo, [22 * CV.KPH_TO_MS, 40 * CV.KPH_TO_MS], [1, 0])
          self.comma_pedal = (gapInterP * self.comma_pedal_original) + ((1.0 - gapInterP) * self.comma_pedal_new)

          actuators.commaPedarlOigin = self.comma_pedal

          # ========================================================================================================= #
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
                    actuators.pedalDistanceAdder = interp(d, [1, 10, 15, 30], [-0.0250, -0.0075, 0.0175, 0.1000])
                  actuators.pedalAdderFinal = (actuators.pedalStartingAdder + actuators.pedalDistanceAdder)

                if self.stoppingStateTimeWindowsActiveCounter > (stoppingStateWindowsActiveCounterLimits) \
                        or (controls.LoC.long_control_state == LongCtrlState.stopping) \
                        or CS.out.vEgo > 35 * CV.KPH_TO_MS \
                        or controls.LoC.pid.f < -0.65:

                  if controls.LoC.pid.f < -0.625:
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
              self.comma_pedal = min(self.comma_pedal, 0.2975)

          # braking logic
          if actuators.accel < -0.125:
            #can_sends.append(gmcan.create_regen_paddle_command(self.packer_pt, CanBus.POWERTRAIN))
            #actuators.regenPaddle = True  # for icon
            self.comma_pedal *= 0.95
          elif controls.LoC.pid.f < - 0.625:
            #can_sends.append(gmcan.create_regen_paddle_command(self.packer_pt, CanBus.POWERTRAIN))
            #actuators.regenPaddle = True  # for icon
            minMultipiler = interp(CS.out.vEgo,
                                    [20 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 120 * CV.KPH_TO_MS],
                                   [0.825, 0.725, 0.600, 0.100])
            self.comma_pedal *= interp(controls.LoC.pid.f, [-2.25, -2.0, -1.5, -0.625],
                                       [0, 0.020, minMultipiler, 0.875])
          actuators.commaPedal = self.comma_pedal

        else:
          self.comma_pedal = 0.0  # Must be set by zero, otherwise cannot re-acceling when stopped. - jc01rho.
          actuators.commaPedal = self.comma_pedal

        # ========================================================================================================= #

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
