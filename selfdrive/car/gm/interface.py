#!/usr/bin/env python3
from cereal import car
from common.numpy_fast import interp
from math import fabs
from common.conversions import Conversions as CV
from selfdrive.car.gm.values import CAR, HIGH_TORQUE, CruiseButtons, \
                                    AccState, CarControllerParams, NO_ASCM
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MIN
from common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter

class CarInterface(CarInterfaceBase):

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    params = CarControllerParams(CP)
    return params.ACCEL_MIN, params.ACCEL_MAX




  # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_volt(desired_angle, v_ego):
    desired_angle *= 0.02904609
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.10006696 * sigmoid * (v_ego + 3.12485927)

  @staticmethod
  def get_steer_feedforward_acadia(desired_angle, v_ego):
    desired_angle *= 0.09760208
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.04689655 * sigmoid * (v_ego + 10.028217)

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint == CAR.VOLT:
      return self.get_steer_feedforward_volt
    elif self.CP.carFingerprint == CAR.ACADIA:
      return self.get_steer_feedforward_acadia
    else:
      return CarInterfaceBase.get_steer_feedforward_default

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, disable_radar=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "gm"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
    ret.alternativeExperience = 1 # UNSAFE_DISABLE_DISENGAGE_ON_GAS
    ret.pcmCruise = False  # stock cruise control is kept off
    ret.openpilotLongitudinalControl = True # ASCM vehicles use OP for long
    ret.radarOffCan = False # ASCM vehicles (typically) have radar

     # Default to normal torque limits
    ret.safetyConfigs[0].safetyParam = 0

    # Start with a baseline lateral tuning for all GM vehicles. Override tuning as needed in each model section below.
    ret.enableGasInterceptor = 0x201 in fingerprint[0]

    if ret.enableGasInterceptor:
      ret.openpilotLongitudinalControl = True

    ret.minSteerSpeed = 11 * CV.KPH_TO_MS
    ret.minEnableSpeed = -1
    ret.mass = 1645. + STD_CARGO_KG
    ret.wheelbase = 2.725
    ret.centerToFront = ret.wheelbase * 0.49  # wild guess
    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.torque

    tire_stiffness_factor = 1.
    ret.maxSteeringAngleDeg = 1000.

    ret.lateralTuning.init('lqr')
    ret.lateralTuning.lqr.scale = 1700.0
    ret.lateralTuning.lqr.ki = 0.03
    ret.lateralTuning.lqr.dcGain = 0.003
    ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
    ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
    ret.lateralTuning.lqr.c = [1., 0.]
    ret.lateralTuning.lqr.k = [-105.0, 450.0]
    ret.lateralTuning.lqr.l = [0.22, 0.318]

    ret.steerRatio = 17.5
    # steerActuatorDelay, steerMaxV 커질수록 인으로 붙고, scale 작을수록 인으로 붙는다.
    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.35
    ret.steerLimitTimer = 0.4   # steerLimitAlert 가 발행되기 전의 시간 (핸들 조향을 하는데 100을 하라고 명령을 했는데, 그걸 해내는데 리미트 시간)

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.longitudinalTuning.kpBP = [0., 5., 35.]
    ret.longitudinalTuning.kpV = [1.1, 0.75, 0.5]
    ret.longitudinalTuning.kiBP = [0., 35.]
    ret.longitudinalTuning.kiV = [0.18, 0.12]

    #ret.longitudinalActuatorDelayLowerBound = 0.3   # 추정 자동차 특정 지연, 지금은 0.3초 사용 (2.5초 까지 미래에 대한 예상 움직임이 나온다)
    #ret.longitudinalActuatorDelayUpperBound = 0.3

    ret.stoppingControl = False   # 자동차가 정지할 때 저속에서도 완전히 제어 여부
    #ret.vEgoStopping = V_CRUISE_MIN / CV.MS_TO_KPH
    #ret.vEgoStarting = V_CRUISE_MIN / CV.MS_TO_KPH

    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

    return ret

  def _update(self, c: car.CarControl) -> car.CarState:
    pass

  # returns a car.CarState
  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_loopback.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_loopback)

    ret.canValid = self.cp.can_valid and self.cp_loopback.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    ret.cruiseState.enabled = ret.cruiseState.available

    buttonEvents = []

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != CruiseButtons.UNPRESS:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        if not (ret.cruiseState.enabled and ret.standstill):
          be.type = ButtonType.accelCruise  # Suppress resume button if we're resuming from stop so we don't adjust speed.
      elif but == CruiseButtons.DECEL_SET:
        be.type = ButtonType.decelCruise
      elif but == CruiseButtons.CANCEL:
        be.type = ButtonType.cancel
      elif but == CruiseButtons.MAIN:
        be.type = ButtonType.altButton3
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents
    # TODO: JJS Move this to appropriate place (check other brands)
    EXTRA_GEARS = [GearShifter.sport, GearShifter.low, GearShifter.eco, GearShifter.manumatic]
    events = self.create_common_events(ret, extra_gears = EXTRA_GEARS, pcm_enable=self.CS.CP.pcmCruise)

    #if ret.vEgo < self.CP.minEnableSpeed:
    #  events.add(EventName.belowEngageSpeed)
    #if self.CS.park_brake:
    #  events.add(EventName.parkBrake)
    #if ret.cruiseState.standstill:
    #  events.add(EventName.resumeRequired)
    #if (self.CS.CP.carFingerprint not in NO_ASCM) and self.CS.pcm_acc_status == AccState.FAULTED:
    #  events.add(EventName.accFaulted)
    #if ret.vEgo < self.CP.minSteerSpeed:
    #  events.add(car.CarEvent.EventName.belowSteerSpeed)

    # handle button presses
    #for b in ret.buttonEvents:
      # do enable on both accel and decel buttons
    #  if b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
    #    events.add(EventName.buttonEnable)
      # do disable on button down
    #  if b.type == ButtonType.cancel and b.pressed:
    #    events.add(EventName.buttonCancel)

    ###

    if self.CP.enableGasInterceptor:
      if not self.CS.main_on:  # lat dis-engage
        for b in ret.buttonEvents:
          if (b.type == ButtonType.decelCruise and not b.pressed) and not self.CS.adaptive_Cruise:
            self.CS.adaptive_Cruise = True
            self.CS.enable_lkas = True
            events.add(EventName.buttonEnable)
            break
          if (b.type == ButtonType.accelCruise and not b.pressed) and not self.CS.adaptive_Cruise:
            self.CS.adaptive_Cruise = True
            self.CS.enable_lkas = True
            events.add(EventName.buttonEnable)
            break
          if (b.type == ButtonType.cancel and b.pressed) and self.CS.adaptive_Cruise:
            self.CS.adaptive_Cruise = False
            self.CS.enable_lkas = False
            #events.add(EventName.buttonEnable)
            events.add(EventName.buttonCancel)
            break
          if (b.type == ButtonType.altButton3 and b.pressed):  # and self.CS.adaptive_Cruise
            self.CS.adaptive_Cruise = False
            self.CS.enable_lkas = True
            events.add(EventName.buttonEnable)  # 어느 이벤트가 먼저인지 확인
            break
      else:  # lat engage
        self.CS.adaptive_Cruise = False
        self.CS.enable_lkas = True

    else:
      if self.CS.main_on:  # wihtout pedal case
        self.CS.adaptive_Cruise = False
        self.CS.enable_lkas = True


    # Added by jc01rho inspired by JangPoo
    #if self.CS.main_on and self.CS.enable_lkas and not self.CS.adaptive_Cruise and ret.cruiseState.enabled and ret.gearShifter == GearShifter.drive and ret.vEgo > 2 and not ret.brakePressed:
    """if not self.CS.main_on and ret.cruiseState.enabled and ret.gearShifter == GearShifter.drive and ret.vEgo > 2:
      if ret.cruiseState.available and not ret.seatbeltUnlatched and not ret.espDisabled and self.flag_pcmEnable_able:

        if self.flag_pcmEnable_initialSet == False:
          self.initial_pcmEnable_counter = self.initial_pcmEnable_counter + 1
          if self.initial_pcmEnable_counter > 750:
            # events.add(EventName.pcmEnable)
            # self.flag_pcmEnable_initialSet = True
            self.flag_pcmEnable_able = False
            self.initial_pcmEnable_counter = 0
        else:
          events.add(EventName.pcmEnable)
          self.flag_pcmEnable_able = False
          # self.flag_pcmEnable_initialSet = True
          # self.initial_pcmEnable_counter = 0
    else:
      self.flag_pcmEnable_able = True
    """

    ###

    ret.events = events.to_msg()

    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    hud_v_cruise = hud_control.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    # For Openpilot, "enabled" includes pre-enable.
    # In GM, PCM faults out if ACC command overlaps user gas.
    # Does not apply when no built-in ACC
    # TODO: This isn't working right... should maybe use unsafe blah blah
    # pedal was disengaging
    if not self.CP.enableGasInterceptor or self.CP.carFingerprint in NO_ASCM:
      enabled = c.enabled # and not self.CS.out.gasPressed
    else:
      enabled = c.enabled

    ret = self.CC.update(c, enabled, self.CS, self.frame,
                         c.actuators,
                         hud_v_cruise, hud_control.lanesVisible,
                         hud_control.leadVisible, hud_control.visualAlert)

    self.frame += 1
    return ret
