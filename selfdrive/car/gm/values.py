from cereal import car
from selfdrive.car import dbc_dict

Ecu = car.CarParams.Ecu
from common.conversions import Conversions as CV

SLOW_ON_CURVES = 1  # 슬로우 커브 사용 유무 (0,1)
MIN_CURVE_SPEED = 32. * CV.KPH_TO_MS

# m/s 속도로 1초당 1미터 -> 3초면 3미터를 이동
# 2 m/s^2는 가속도 1초당 2m/s 속도 증가 -> 1초면 2미터 이동 -> <1초면 4미터 이동>

class CarControllerParams():

    STEER_MAX = 300  # GM limit is 3Nm. Used by carcontroller to generate LKA output
    STEER_STEP = 2  # Control frames per command (50hz)
    STEER_DELTA_UP = 7  # Delta rates require review due to observed EPS weakness
    STEER_DELTA_DOWN = 17
    STEER_DRIVER_ALLOWANCE = 50
    STEER_DRIVER_MULTIPLIER = 4
    STEER_DRIVER_FACTOR = 100
    NEAR_STOP_BRAKE_PHASE = 0.5  # m/s

    # Heartbeat for dash "Service Adaptive Cruise" and "Service Front Camera"
    ADAS_KEEPALIVE_STEP = 100
    CAMERA_KEEPALIVE_STEP = 100

    # Volt gasbrake lookups
    # TODO: These values should be confirmed on non-Volt vehicles
    MAX_GAS = 3072  # Safety limit, not ACC max. Stock ACC >4096 from standstill.
    ZERO_GAS = 2048  # Coasting
    MAX_BRAKE = 350  # ~ -3.5 m/s^2 with regen
    MAX_ACC_REGEN = 1404  # Max ACC regen is slightly less than max paddle regen

    # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
    # perform the closed loop control, and might need some
    # to apply some more braking if we're on a downhill slope.
    # Our controller should still keep the 2 second average above
    # -3.5 m/s^2 as per planner limits
    ACCEL_MAX = 2.  # m/s^2
    ACCEL_MIN = -4.  # m/s^2

    EV_GAS_LOOKUP_BP = [-1., 0., ACCEL_MAX]
    EV_BRAKE_LOOKUP_BP = [ACCEL_MIN, -1.]

    # ICE has much less engine braking force compared to regen in EVs,
    # lower threshold removes some braking deadzone
    GAS_LOOKUP_BP = [-0.1, 0., ACCEL_MAX]
    BRAKE_LOOKUP_BP = [ACCEL_MIN, -0.1]

    GAS_LOOKUP_V = [MAX_ACC_REGEN, ZERO_GAS, MAX_GAS]
    BRAKE_LOOKUP_V = [MAX_BRAKE, 0.]


class CAR:
    EQUINOX_NR = "CHEVROLET EQUINOX NO RADAR"


class CruiseButtons:
    INIT = 0
    UNPRESS = 1
    RES_ACCEL = 2
    DECEL_SET = 3
    MAIN = 5
    CANCEL = 6


class AccState:
    OFF = 0
    ACTIVE = 1
    FAULTED = 3
    STANDSTILL = 4


# TODO: update these values for Direct OBD w ASCM and without, Universal harness w and wo ascm, cam harness
class CanBus:
    POWERTRAIN = 0
    OBSTACLE = 1
    CHASSIS = 2
    SW_GMLAN = 3
    LOOPBACK = 128


FINGERPRINTS = {

    CAR.EQUINOX_NR: [
        # Equinox w/o ACC 2020 + Pedal (512: 6, 513: 6)
        {
            190: 6, 193: 8, 197: 8, 199: 4, 201: 8, 209: 7, 211: 2, 241: 6, 249: 8, 257: 5, 288: 5, 289: 8, 298: 8,
            304: 1, 309: 8, 311: 8, 313: 8, 320: 3, 322: 7, 328: 1, 352: 5, 381: 8, 384: 4, 386: 8, 388: 8, 393: 8,
            398: 8, 401: 8, 413: 8, 417: 7, 419: 1, 422: 4, 426: 7, 431: 8, 442: 8, 451: 8, 452: 8, 453: 6, 455: 7,
            456: 8, 479: 3, 481: 7, 485: 8, 489: 8, 497: 8, 499: 3, 500: 6, 501: 8, 508: 8, 510: 8, 512: 6, 513: 6,
            528: 5, 532: 6, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 565: 5, 567: 5, 569: 3, 573: 1, 577: 8, 647: 6,
            707: 8, 715: 8, 717: 5, 730: 4, 753: 5, 761: 7, 789: 5, 800: 6, 806: 1, 810: 8, 840: 5, 842: 5, 844: 8,
            866: 4, 869: 4, 880: 6, 882: 8, 890: 1, 892: 2, 893: 2, 894: 1, 961: 8, 969: 8, 977: 8, 979: 8, 985: 5,
            1001: 8, 1009: 8, 1011: 6, 1017: 8, 1020: 8, 1033: 7, 1034: 7, 1105: 6, 1217: 8, 1219: 8, 1221: 5, 1225: 8,
            1231: 3, 1233: 8, 1249: 8, 1257: 6, 1259: 8, 1261: 7, 1263: 4, 1265: 8, 1267: 1, 1271: 8, 1273: 3, 1280: 4,
            1296: 4, 1300: 8, 1322: 6, 1328: 4, 1345: 8, 1346: 8, 1347: 5, 1353: 8, 1355: 8, 1362: 8, 1375: 8, 1417: 8,
            1512: 8, 1514: 8, 1601: 8, 1602: 8, 1906: 8, 1907: 7, 1912: 7, 1916: 7, 1919: 7, 1920: 7, 1930: 7, 2016: 8,
            2024: 8
        }],

}

DBC = {
    CAR.EQUINOX_NR: dbc_dict('gm_global_a_powertrain_generated', 'gm_global_a_object', chassis_dbc='gm_global_a_chassis'),
}

STEER_THRESHOLD = 1.0