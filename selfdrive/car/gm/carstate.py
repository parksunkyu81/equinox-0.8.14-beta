from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, EV_CAR, AccState, CanBus, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL2"]["PRNDL2"]
    self.lka_steering_cmd_counter = 0
    self.park_brake = 0

    self.adaptive_Cruise = False
    self.enable_lkas = False
    self.main_on = False

  def update(self, pt_cp, loopback_cp):
    ret = car.CarState.new_message()

    ret.adaptiveCruise = self.adaptive_Cruise
    ret.lkasEnable = self.enable_lkas

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01
    gmGear = self.shifter_values.get(pt_cp.vl["ECMPRDNL2"]["PRNDL2"], None)
    manualMode = bool(pt_cp.vl["ECMPRDNL2"]["ManualMode"])
    
    # In manumatic, the value of PRNDL2 represents the max gear
    # parse_gear_shifter expects "T" for manumatic
    # TODO: JJS Add manumatic max gear to cereal and parse (validate value in Acadia)
    if manualMode:
      gmGear = "T"
    
    # TODO: JJS: Should We just have 0 map to P in the dbc?
    if gmGear == "Shifting":
      gmGear = "P"
    
    ret.gearShifter = self.parse_gear_shifter(gmGear)
    ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0
    ret.brake = 0.
    
    if ret.brakePressed:
      ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0

    if self.CP.enableGasInterceptor:
      ret.gas = (pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254.
      ret.gasPressed = ret.gas > 1e-5

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    self.lka_steering_cmd_counter = loopback_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]

    # 0 inactive, 1 active, 2 temporarily limited, 3 failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]["LKATorqueDeliveredStatus"]
    ret.steerFaultTemporary = self.lkas_status == 2
    ret.steerFaultPermanent = self.lkas_status == 3

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2
    #TODO: JJS: Add hasEPB to cereal and use detection rather than hard coding...
    #if self.CP.hasEPB:
    self.park_brake = pt_cp.vl["EPBStatus"]["EPBClosed"]
    ret.autoHold = pt_cp.vl["EPBStatus"]["EPBClosed"]

    ret.cruiseState.available = pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"] != 0

    # 크루즈 메인버튼 활성 여부
    self.main_on = bool(pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"])
    ret.mainOn = self.main_on

    if self.CP.enableGasInterceptor: # Flip CC main logic when pedal is being used for long
      ret.cruiseState.available = (not ret.cruiseState.available)

    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]["CruiseState"]

    #ret.cruiseState.enabled = self.pcm_acc_status != AccState.OFF
    #ret.cruiseState.standstill = self.pcm_acc_status == AccState.STANDSTILL

    ret.cruiseState.available = self.pcm_acc_status != 0
    ret.cruiseState.standstill = False

    ret.cruiseState.enabled = self.main_on or ret.adaptiveCruise

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("BrakePedalPosition", "EBCMBrakePedalPosition"),
      ("FrontLeftDoor", "BCMDoorBeltStatus"),
      ("FrontRightDoor", "BCMDoorBeltStatus"),
      ("RearLeftDoor", "BCMDoorBeltStatus"),
      ("RearRightDoor", "BCMDoorBeltStatus"),
      ("LeftSeatBelt", "BCMDoorBeltStatus"),
      ("RightSeatBelt", "BCMDoorBeltStatus"),
      ("TurnSignals", "BCMTurnSignals"),
      ("AcceleratorPedal2", "AcceleratorPedal2"),
      ("CruiseState", "AcceleratorPedal2"),
      ("ACCButtons", "ASCMSteeringButton"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("FLWheelSpd", "EBCMWheelSpdFront"),
      ("FRWheelSpd", "EBCMWheelSpdFront"),
      ("RLWheelSpd", "EBCMWheelSpdRear"),
      ("RRWheelSpd", "EBCMWheelSpdRear"),
      ("PRNDL2", "ECMPRDNL2"),
      ("ManualMode", "ECMPRDNL2"),
      ("LKADriverAppldTrq", "PSCMStatus"),
      ("LKATorqueDelivered", "PSCMStatus"),
      ("LKATorqueDeliveredStatus", "PSCMStatus"),
      ("DistanceButton", "ASCMSteeringButton"),
      ("TractionControlOn", "ESPStatus"),
      ("CruiseMainOn", "ECMEngineStatus"),
      ("Brake_Pressed", "ECMEngineStatus"),
    ]

    checks = [
      ("BCMTurnSignals", 1),
      ("ECMPRDNL2", 10),
      ("PSCMStatus", 10),
      ("ESPStatus", 10),
      ("BCMDoorBeltStatus", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("AcceleratorPedal2", 33),
      ("ASCMSteeringButton", 33),
      ("ECMEngineStatus", 100),
      ("PSCMSteeringAngle", 100),
      ("EBCMBrakePedalPosition", 100),
    ]

    # TODO: Might be wise to find the non-electronic parking brake signal
    # TODO: JJS Add hasEPB to cereal
    if CP.carFingerprint != CAR.SUBURBAN and CP.carFingerprint != CAR.TAHOE_NR:
      signals.append(("EPBClosed", "EPBStatus", 0))
      checks.append(("EPBStatus", 20))
    

    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR"))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR"))
      checks.append(("GAS_SENSOR", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("RollingCounter", "ASCMLKASteeringCmd"),
    ]

    checks = [
      ("ASCMLKASteeringCmd", 50),  # DEF : 50
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK)
