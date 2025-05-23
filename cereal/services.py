#!/usr/bin/env python3
import os
from typing import Optional

TICI = os.path.isfile('/TICI')
RESERVED_PORT = 8022  # sshd
STARTING_PORT = 8001


def new_port(port: int):
  port += STARTING_PORT
  return port + 1 if port >= RESERVED_PORT else port


class Service:
  def __init__(self, port: int, should_log: bool, frequency: float, decimation: Optional[int] = None):
    self.port = port
    self.should_log = should_log
    self.frequency = frequency
    self.decimation = decimation

DCAM_FREQ = 10. if not TICI else 20.

services = {
  # service: (should_log, frequency, qlog decimation (optional))
  # note: the "EncodeIdx" packets will still be in the log
  "sensorEvents": (True, 100., 100),
  "gpsNMEA": (True, 9.),
  "deviceState": (True, 2., 1),
  "can": (True, 100.),
  "controlsState": (True, 100., 10),
  "pandaStates": (True, 2., 1),
  "peripheralState": (True, 2., 1),
  "radarState": (True, 20., 5),
  "roadEncodeIdx": (False, 20., 1),
  "liveTracks": (True, 20.),
  "sendcan": (True, 100., 139),
  "logMessage": (True, 0.),
  "errorLogMessage": (True, 0., 1),
  "liveCalibration": (True, 4., 4),
  "androidLog": (True, 0.),
  "carState": (True, 100., 10),
  "carControl": (True, 100., 10),
  "longitudinalPlan": (True, 20., 5),
  "procLog": (True, 0.5),
  "gpsLocationExternal": (True, 10., 10),
  "ubloxGnss": (True, 10.),
  "qcomGnss": (True, 2.),
  "clocks": (True, 1., 1),
  "ubloxRaw": (True, 20.),
  "liveLocationKalman": (True, 20., 5),
  "liveParameters": (True, 20., 5),
  "cameraOdometry": (True, 20., 5),
  "lateralPlan": (True, 20., 5),
  "thumbnail": (True, 0.2, 1),
  "carEvents": (True, 1., 1),
  "carParams": (True, 0.02, 1),
  "roadCameraState": (True, 20., 20),
  "driverCameraState": (True, DCAM_FREQ, DCAM_FREQ),
  "driverEncodeIdx": (False, DCAM_FREQ, 1),
  "driverState": (True, DCAM_FREQ, DCAM_FREQ / 2),
  "driverMonitoringState": (True, DCAM_FREQ, DCAM_FREQ / 2),
  "wideRoadEncodeIdx": (False, 20., 1),
  "wideRoadCameraState": (True, 20., 20),
  "modelV2": (True, 20., 40),
  "managerState": (True, 2., 1),
  "uploaderState": (True, 0., 1),
  "navInstruction": (True, 0., 10),
  "navRoute": (True, 0.),
  "navThumbnail": (True, 0.),
  "roadLimitSpeed": (False, 0.),
  "liveTorqueParameters": (True, 4., 1),
  # shane
  "dynamicFollowData": (False, 20.),

  # debug
  "testJoystick": (False, 0.),
  "roadEncodeData": (False, 20.),
  "driverEncodeData": (False, DCAM_FREQ),
  "wideRoadEncodeData": (False, 20.),
}
service_list = {name: Service(new_port(idx), *vals) for  # type: ignore
                idx, (name, vals) in enumerate(services.items())}


def build_header():
  h = ""
  h += "/* THIS IS AN AUTOGENERATED FILE, PLEASE EDIT services.py */\n"
  h += "#ifndef __SERVICES_H\n"
  h += "#define __SERVICES_H\n"
  h += "struct service { char name[0x100]; int port; bool should_log; int frequency; int decimation; };\n"
  h += "static struct service services[] = {\n"
  for k, v in service_list.items():
    should_log = "true" if v.should_log else "false"
    decimation = -1 if v.decimation is None else v.decimation
    h += '  { "%s", %d, %s, %d, %d },\n' % \
         (k, v.port, should_log, v.frequency, decimation)
  h += "};\n"
  h += "#endif\n"
  return h


if __name__ == "__main__":
  print(build_header())
