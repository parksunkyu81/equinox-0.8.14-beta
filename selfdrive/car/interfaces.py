import yaml
import os
import time
from abc import abstractmethod, ABC
from typing import Any, Dict, Optional, Tuple, List, Callable
from common.numpy_fast import interp

from cereal import car
from common.basedir import BASEDIR
from common.kalman.simple_kalman import KF1D
from common.realtime import DT_CTRL
from selfdrive.car import gen_empty_fingerprint
from common.conversions import Conversions as CV
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, apply_deadzone
from selfdrive.controls.lib.events import Events
from selfdrive.controls.lib.vehicle_model import VehicleModel

GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
TorqueFromLateralAccelCallbackType = Callable[[float, car.CarParams.LateralTorqueTuning, float, float, bool], float]

MAX_CTRL_SPEED = (V_CRUISE_MAX + 4) * CV.KPH_TO_MS
ACCEL_MAX = 2.0
ACCEL_MIN = -3.5
FRICTION_THRESHOLD = 0.2

# generic car and radar interfaces

class CarInterfaceBase(ABC):
  def __init__(self, CP, CarController, CarState):
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.steering_unpressed = 0
    self.low_speed_alert = False
    self.silent_steer_warning = True
    self.v_ego_cluster_seen = False

    self.CS = None
    self.can_parsers = []
    if CarState is not None:
      self.CS = CarState(CP)

      self.cp = self.CS.get_can_parser(CP)
      self.cp_cam = self.CS.get_cam_can_parser(CP)
      self.cp_adas = self.CS.get_adas_can_parser(CP)
      self.cp_body = self.CS.get_body_can_parser(CP)
      self.cp_loopback = self.CS.get_loopback_can_parser(CP)
      self.can_parsers = [self.cp, self.cp_cam, self.cp_adas, self.cp_body, self.cp_loopback]

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP, self.VM)

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return ACCEL_MIN, ACCEL_MAX

  @staticmethod
  @abstractmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, experimental_long=False):
    pass

  @staticmethod
  def init(CP, logcan, sendcan):
    pass

  @staticmethod
  def get_steer_feedforward_default(desired_angle, v_ego):
    # Proportional to realigning tire momentum: lateral acceleration.
    # TODO: something with lateralPlan.curvatureRates
    return desired_angle * (v_ego**2)

  def get_steer_feedforward_function(self):
    return self.get_steer_feedforward_default

  @staticmethod
  def torque_from_lateral_accel_linear(lateral_accel_value,
                                       torque_params,
                                       lateral_accel_error=None,
                                       lateral_accel_deadzone=None,
                                       friction_compensation=False):
    # The default is a linear relationship between torque and lateral acceleration (accounting for road roll and steering friction)
    if friction_compensation:
      assert (lateral_accel_error is not None) and (lateral_accel_deadzone is not None)
      friction = interp(
        apply_deadzone(lateral_accel_error, lateral_accel_deadzone),
        [-FRICTION_THRESHOLD, FRICTION_THRESHOLD],
        [-torque_params['friction'], torque_params['friction']]
      )
    else:
      friction = 0.0
    return (lateral_accel_value / torque_params['latAccelFactor']) + friction

  def torque_from_lateral_accel(self):
    return self.torque_from_lateral_accel_linear


  # returns a set of default params to avoid repetition in car specific params
  @staticmethod
  def get_std_params(candidate, fingerprint):
    ret = car.CarParams.new_message()
    ret.carFingerprint = candidate

    # standard ALC params
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.minSteerSpeed = 0.
    ret.wheelSpeedFactor = 1.0
    ret.maxLateralAccel = 1.8
    ret.pcmCruise = True     # openpilot's state is tied to the PCM's cruise state on most cars
    ret.minEnableSpeed = -1. # enable is done by stock ACC, so ignore this
    ret.steerRatioRear = 0.  # no rear steering, at least on the listed cars aboveA
    ret.openpilotLongitudinalControl = False
    ret.stopAccel = -2.0
    ret.stoppingDecelRate = 0.8 # brake_travel/s while trying to stop
    ret.vEgoStopping = 0.5
    ret.vEgoStarting = 0.5
    ret.stoppingControl = True
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kf = 1.
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [1.]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [1.]
    # TODO estimate car specific lag, use .15s for now
    ret.longitudinalActuatorDelayLowerBound = 0.15
    ret.longitudinalActuatorDelayUpperBound = 0.15
    ret.steerLimitTimer = 1.0
    return ret

  @staticmethod
  def configure_torque_tune(tune, LAT_ACCEL_FACTOR=2.4, FRICTION=0.175, steering_angle_deadzone_deg=0.03, use_steering_angle=True):
    tune.init('torque')
    tune.torque.useSteeringAngle = use_steering_angle
    tune.torque.kp = 1.0
    tune.torque.kf = 1.0
    tune.torque.ki = 0.1
    tune.torque.friction = FRICTION
    tune.torque.latAccelFactor = LAT_ACCEL_FACTOR
    tune.torque.latAccelOffset = 0.0
    tune.torque.steeringAngleDeadzoneDeg = steering_angle_deadzone_deg

  @abstractmethod
  def _update(self, c: car.CarControl) -> car.CarState:
    pass

  def update(self, c: car.CarControl, can_strings: List[bytes]) -> car.CarState:
    # parse can
    for cp in self.can_parsers:
      if cp is not None:
        cp.update_strings(can_strings)

    # get CarState
    ret = self._update(c)

    ret.canValid = all(cp.can_valid for cp in self.can_parsers if cp is not None)
    ret.canTimeout = any(cp.bus_timeout for cp in self.can_parsers if cp is not None)

    # copy back for next iteration
    reader = ret.as_reader()
    if self.CS is not None:
      self.CS.out = reader

    return reader

  @abstractmethod
  def apply(self, c: car.CarControl, controls) -> Tuple[car.CarControl.Actuators, List[bytes]]:
    pass

  def create_common_events(self, cs_out, extra_gears=None, pcm_enable=True):
    events = Events()


    # janpoo6427
    if cs_out.gearShifter != GearShifter.park:
      if cs_out.doorOpen:
        events.add(EventName.doorOpen)
      if cs_out.seatbeltUnlatched:
        events.add(EventName.seatbeltNotLatched)
    if cs_out.gearShifter != GearShifter.drive and (extra_gears is None or
       cs_out.gearShifter not in extra_gears):
      events.add(EventName.wrongGear)
    if cs_out.gearShifter == GearShifter.reverse:
      events.add(EventName.reverseGear)
    if not cs_out.cruiseState.available:
      events.add(EventName.wrongCarMode)
    if cs_out.espDisabled:
      events.add(EventName.espDisabled)
    if cs_out.stockFcw:
      events.add(EventName.stockFcw)
    if cs_out.stockAeb:
      events.add(EventName.stockAeb)
    if cs_out.vEgo > MAX_CTRL_SPEED:
      events.add(EventName.speedTooHigh)
    if cs_out.cruiseState.nonAdaptive:
      events.add(EventName.wrongCruiseMode)
    #if cs_out.brakeHoldActive and self.CP.openpilotLongitudinalControl:
    #  events.add(EventName.brakeHold)
    #if cs_out.parkingBrake:
    #  events.add(EventName.parkBrake)

    # Handle permanent and temporary steering faults
    self.steering_unpressed = 0 if cs_out.steeringPressed else self.steering_unpressed + 1
    if cs_out.steerFaultTemporary:
      # if the user overrode recently, show a less harsh alert
      if self.silent_steer_warning or cs_out.standstill or self.steering_unpressed < int(1.5 / DT_CTRL):
        self.silent_steer_warning = True
        events.add(EventName.steerTempUnavailableSilent)
      else:
        events.add(EventName.steerTempUnavailable)
    else:
      self.silent_steer_warning = False
    if cs_out.steerFaultPermanent:
      events.add(EventName.steerUnavailable)

    # we engage when pcm is active (rising edge)
    if pcm_enable:
      if cs_out.cruiseState.enabled and not self.CS.out.cruiseState.enabled:
        events.add(EventName.pcmEnable)
      elif not cs_out.cruiseState.enabled:
        events.add(EventName.pcmDisable)

    # janpoo6427
    # move to controlsd

    return events


class RadarInterfaceBase(ABC):
  def __init__(self, CP):
    self.pts = {}
    self.delay = 0
    self.radar_ts = CP.radarTimeStep
    self.no_radar_sleep = 'NO_RADAR_SLEEP' in os.environ

  def update(self, can_strings):
    ret = car.RadarData.new_message()
    if not self.no_radar_sleep:
      time.sleep(self.radar_ts)  # radard runs on RI updates
    return ret


class CarStateBase(ABC):
  def __init__(self, CP):
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.out = car.CarState.new_message()

    self.cruise_buttons = 0
    self.left_blinker_cnt = 0
    self.right_blinker_cnt = 0
    self.left_blinker_prev = False
    self.right_blinker_prev = False

    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, DT_CTRL], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])

    self.v_ego_clu_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, DT_CTRL], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])

  def update_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_ego_raw], [0.0]]

    v_ego_x = self.v_ego_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])

  def update_clu_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_clu_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_clu_kf.x = [[v_ego_raw], [0.0]]

    v_ego_x = self.v_ego_clu_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])

  def get_wheel_speeds(self, fl, fr, rl, rr, unit=CV.KPH_TO_MS):
    factor = unit * self.CP.wheelSpeedFactor

    wheelSpeeds = car.CarState.WheelSpeeds.new_message()
    wheelSpeeds.fl = fl * factor
    wheelSpeeds.fr = fr * factor
    wheelSpeeds.rl = rl * factor
    wheelSpeeds.rr = rr * factor
    return wheelSpeeds

  def update_blinker_from_lamp(self, blinker_time: int, left_blinker_lamp: bool, right_blinker_lamp: bool):
    """Update blinkers from lights. Enable output when light was seen within the last `blinker_time`
    iterations"""
    # TODO: Handle case when switching direction. Now both blinkers can be on at the same time
    self.left_blinker_cnt = blinker_time if left_blinker_lamp else max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = blinker_time if right_blinker_lamp else max(self.right_blinker_cnt - 1, 0)
    return self.left_blinker_cnt > 0, self.right_blinker_cnt > 0

  def update_blinker_from_stalk(self, blinker_time: int, left_blinker_stalk: bool, right_blinker_stalk: bool):
    """Update blinkers from stalk position. When stalk is seen the blinker will be on for at least blinker_time,
    or until the stalk is turned off, whichever is longer. If the opposite stalk direction is seen the blinker
    is forced to the other side. On a rising edge of the stalk the timeout is reset."""

    if left_blinker_stalk:
      self.right_blinker_cnt = 0
      if not self.left_blinker_prev:
        self.left_blinker_cnt = blinker_time

    if right_blinker_stalk:
      self.left_blinker_cnt = 0
      if not self.right_blinker_prev:
        self.right_blinker_cnt = blinker_time

    self.left_blinker_cnt = max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = max(self.right_blinker_cnt - 1, 0)

    self.left_blinker_prev = left_blinker_stalk
    self.right_blinker_prev = right_blinker_stalk

    return bool(left_blinker_stalk or self.left_blinker_cnt > 0), bool(right_blinker_stalk or self.right_blinker_cnt > 0)

  @staticmethod
  def parse_gear_shifter(gear: str) -> car.CarState.GearShifter:
    d: Dict[str, car.CarState.GearShifter] = {
        'P': GearShifter.park, 'R': GearShifter.reverse, 'N': GearShifter.neutral,
        'E': GearShifter.eco, 'T': GearShifter.manumatic, 'D': GearShifter.drive,
        'S': GearShifter.sport, 'L': GearShifter.low, 'B': GearShifter.brake
    }
    return d.get(gear, GearShifter.unknown)

  @staticmethod
  def get_cam_can_parser(CP):
    return None

  @staticmethod
  def get_adas_can_parser(CP):
    return None

  @staticmethod
  def get_body_can_parser(CP):
    return None

  @staticmethod
  def get_loopback_can_parser(CP):
    return None
