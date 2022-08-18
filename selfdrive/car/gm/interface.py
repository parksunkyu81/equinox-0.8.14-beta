#!/usr/bin/env python3
from cereal import car
from common.numpy_fast import interp
from math import fabs
from common.conversions import Conversions as CV
from selfdrive.ntune import ntune_scc_get
from selfdrive.car.gm.values import CAR, CruiseButtons, CarControllerParams
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, \
    get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter


class CarInterface(CarInterfaceBase):

    @staticmethod
    def get_pid_accel_limits(current_speed, cruise_speed):
        params = CarControllerParams()
        return params.ACCEL_MIN, params.ACCEL_MAX
        #accel_max_bp = [10., 20., 30., 50.]
        #accel_max_v = [1.45, 1.425, 1.455, 1.35]
        #v_current_kph = current_speed * CV.MS_TO_KPH
        #return params.ACCEL_MIN, interp(v_current_kph, accel_max_bp, accel_max_v)

    # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
    def get_steer_feedforward_function(self):
        return CarInterfaceBase.get_steer_feedforward_default

    @staticmethod
    def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, disable_radar=False):
        ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
        ret.carName = "gm"
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
        ret.alternativeExperience = 1  # UNSAFE_DISABLE_DISENGAGE_ON_GAS
        ret.pcmCruise = False  # stock cruise control is kept off
        ret.openpilotLongitudinalControl = True  # ASCM vehicles use OP for long
        ret.radarOffCan = False  # ASCM vehicles (typically) have radar

        # Default to normal torque limits
        ret.safetyConfigs[0].safetyParam = 0

        # Start with a baseline lateral tuning for all GM vehicles. Override tuning as needed in each model section below.
        ret.enableGasInterceptor = 0x201 in fingerprint[0]

        ret.minSteerSpeed = 11 * CV.KPH_TO_MS
        ret.minEnableSpeed = -1
        ret.mass = 1645. + STD_CARGO_KG
        ret.wheelbase = 2.725
        ret.centerToFront = ret.wheelbase * 0.49  # wild guess
        # no rear steering, at least on the listed cars above
        ret.steerRatioRear = 0.
        ret.steerControlType = car.CarParams.SteerControlType.torque

        tire_stiffness_factor = 0.444  # 1. 을 기준으로 줄면 민감(오버), 커지면 둔감(언더) DEF : 0.5
        ret.maxSteeringAngleDeg = 1000.
        #ret.disableLateralLiveTuning = True

        lateral_control = Params().get("LateralControl", encoding='utf-8')
        if lateral_control == 'INDI':
            ret.lateralTuning.init('indi')
            ret.lateralTuning.indi.innerLoopGainBP = [0.]
            ret.lateralTuning.indi.innerLoopGainV = [3.3]
            ret.lateralTuning.indi.outerLoopGainBP = [0.]
            ret.lateralTuning.indi.outerLoopGainV = [2.8]
            ret.lateralTuning.indi.timeConstantBP = [0.]
            ret.lateralTuning.indi.timeConstantV = [1.4]
            ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
            ret.lateralTuning.indi.actuatorEffectivenessV = [1.8]
        elif lateral_control == 'LQR':
            ret.lateralTuning.init('lqr')

            ret.lateralTuning.lqr.scale = 1955.0
            ret.lateralTuning.lqr.ki = 0.0315
            ret.lateralTuning.lqr.dcGain = 0.002237852961363602

            ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
            ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
            ret.lateralTuning.lqr.c = [1., 0.]
            ret.lateralTuning.lqr.k = [-110.73572306, 451.22718255]
            ret.lateralTuning.lqr.l = [0.3233671, 0.3185757]

        else:
            ret.lateralTuning.init('torque')
            ret.lateralTuning.torque.useSteeringAngle = True
            max_lat_accel = 2.275
            ret.lateralTuning.torque.kp = 2.0 / max_lat_accel
            ret.lateralTuning.torque.kf = 1.0 / max_lat_accel
            ret.lateralTuning.torque.ki = 0.19 / max_lat_accel
            ret.lateralTuning.torque.friction = 0.02
            ret.lateralTuning.torque.kd = 1.0

        ret.steerRatio = 16.85

        # steerActuatorDelay, steerMaxV 커질수록 인으로 붙고, scale 작을수록 인으로 붙는다.
        # steerratecost를 높이면 핸들링이 부드러워(둔감)해 집니다. 다시 말해 도로의 작은 변화에 기민하게 반응하지 않게 됩니다.
        # steeractuatordelay는 계산된 주행곡선을 좀더 빠르게 혹은 느리게 반영할지를 결정합니다
        ret.steerActuatorDelay = 0.21  # DEF : 0.1  너무 늦게 선회하면 steerActuatorDelay를 늘립니다.
        ret.steerLimitTimer = 0.4  # steerLimitAlert 가 발행되기 전의 시간 (핸들 조향을 하는데 100을 하라고 명령을 했는데, 그걸 해내는데 리미트 시간)

        # TODO: get actual value, for now starting with reasonable value for
        # civic and scaling by mass and wheelbase
        ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

        # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
        # mass and CG position, so all cars will have approximately similar dyn behaviors
        ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                             tire_stiffness_factor=tire_stiffness_factor)


        # longitudinal
        ret.longitudinalTuning.kpBP = [0., 5. * CV.KPH_TO_MS, 10. * CV.KPH_TO_MS, 30. * CV.KPH_TO_MS,
                                       130. * CV.KPH_TO_MS]
        ret.longitudinalTuning.kpV = [1.2, 1.0, 0.93, 0.88, 0.5]
        ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
        ret.longitudinalTuning.kiV = [0.1, 0.05]
        ret.longitudinalActuatorDelayLowerBound = 0.3
        ret.longitudinalActuatorDelayUpperBound = 0.3

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
        events = self.create_common_events(ret, extra_gears=EXTRA_GEARS, pcm_enable=self.CS.CP.pcmCruise)

        # if ret.vEgo < self.CP.minEnableSpeed:
        #  events.add(EventName.belowEngageSpeed)
        # if self.CS.park_brake:
        #  events.add(EventName.parkBrake)
        # if ret.cruiseState.standstill:
        #  events.add(EventName.resumeRequired)
        # if (self.CS.CP.carFingerprint not in NO_ASCM) and self.CS.pcm_acc_status == AccState.FAULTED:
        #  events.add(EventName.accFaulted)
        # if ret.vEgo < self.CP.minSteerSpeed:
        #  events.add(car.CarEvent.EventName.belowSteerSpeed)

        # handle button presses
        # for b in ret.buttonEvents:
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
                        # events.add(EventName.buttonEnable)
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

        ###

        ret.events = events.to_msg()

        # copy back carState packet to CS
        self.CS.out = ret.as_reader()

        return self.CS.out

    def apply(self, c, controls):
        hud_control = c.hudControl
        hud_v_cruise = hud_control.setSpeed
        if hud_v_cruise > 70:
            hud_v_cruise = 0

        # For Openpilot, "enabled" includes pre-enable.
        # In GM, PCM faults out if ACC command overlaps user gas.
        # Does not apply when no built-in ACC
        # TODO: This isn't working right... should maybe use unsafe blah blah
        # pedal was disengaging
        enabled = c.enabled

        new_actuators, can_sends = self.CC.update(c, enabled, self.CS, self.frame,
                                                  controls,
                                                  c.actuators,
                                                  hud_v_cruise, hud_control.lanesVisible,
                                                  hud_control.leadVisible, hud_control.visualAlert)

        self.frame += 1
        return new_actuators, can_sends
