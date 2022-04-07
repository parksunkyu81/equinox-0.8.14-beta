import math
from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL
from common.conversions import Conversions as CV
from selfdrive.modeld.constants import T_IDXS
from selfdrive.ntune import ntune_common_get

ButtonType = car.CarState.ButtonEvent.Type
ButtonPrev = ButtonType.unknown
ButtonCnt = 0
LongPressed = False

# 경고: 이 값은 모델의 훈련 분포를 기반으로 결정되었으며,
# 이 속도 이상의 모델 예측은 예측할 수 없습니다.

# kph
V_CRUISE_MAX = 145
V_CRUISE_MIN = 25
V_CRUISE_DELTA_MI = 5 * CV.MPH_TO_KPH
V_CRUISE_DELTA_KM = 10
V_CRUISE_ENABLE_MIN = 1
LAT_MPC_N = 16
LON_MPC_N = 32
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  car.CarState.ButtonEvent.Type.accelCruise: math.ceil,
  car.CarState.ButtonEvent.Type.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  car.CarState.ButtonEvent.Type.accelCruise: +1,
  car.CarState.ButtonEvent.Type.decelCruise: -1,
}


class MPC_COST_LAT:
  PATH = 1.0
  HEADING = 1.0
  STEER_RATE = 1.0


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def update_v_cruise(v_cruise_kph, buttonEvents, enabled, metric):
  global ButtonCnt, LongPressed, ButtonPrev
  if enabled:
    if ButtonCnt:
      ButtonCnt += 1
    for b in buttonEvents:
      if b.pressed and not ButtonCnt and (b.type == ButtonType.accelCruise or b.type == ButtonType.decelCruise):
        ButtonCnt = 1
        ButtonPrev = b.type
      elif not b.pressed and ButtonCnt:
        if not LongPressed and b.type == ButtonType.accelCruise:
          v_cruise_kph += 1 if metric else 1 * CV.MPH_TO_KPH
        elif not LongPressed and b.type == ButtonType.decelCruise:
          v_cruise_kph -= 1 if metric else 1 * CV.MPH_TO_KPH
        LongPressed = False
        ButtonCnt = 0
    if ButtonCnt > 70:
      LongPressed = True
      V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
      if ButtonPrev == ButtonType.accelCruise:
        v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
      elif ButtonPrev == ButtonType.decelCruise:
        v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
      ButtonCnt %= 70
    v_cruise_kph = clip(v_cruise_kph, V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph or above probably means we never had a set speed
    if b.type == car.CarState.ButtonEvent.Type.accelCruise and v_cruise_last < 250:
      return v_cruise_last

  return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = ntune_common_get('steerActuatorDelay') + .2
  current_curvature = curvatures[0]
  psi = interp(delay, T_IDXS[:CONTROL_N], psis)
  desired_curvature_rate = curvature_rates[0]

  # MPC는 t_delay 전에 바퀴를 돌고 되돌아갈 계획입니다.
  # 지연 시간이 긴 경우 일부 수정은 명령조차 받지 않습니다.
  # psi는 원하는 곡률의 단순 선형화를 계산합니다.
  curvature_diff_from_psi = psi / (max(v_ego, 1e-1) * delay) - current_curvature
  desired_curvature = current_curvature + 2 * curvature_diff_from_psi

  v_ego = max(v_ego, 0.1)
  max_curvature_rate = MAX_LATERAL_JERK / ((v_ego/2)**2)
  safe_desired_curvature_rate = clip(desired_curvature_rate,
                                          -max_curvature_rate,
                                          max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature,
                                     current_curvature - max_curvature_rate * DT_MDL,
                                     current_curvature + max_curvature_rate * DT_MDL)

  return safe_desired_curvature, safe_desired_curvature_rate
