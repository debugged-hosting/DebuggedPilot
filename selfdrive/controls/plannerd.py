#!/usr/bin/env python3
from cereal import car
from common.params import Params
from common.realtime import Priority, config_rt_process
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.planner import Planner
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.pathplanner import PathPlanner
import cereal.messaging as messaging
import cereal.messaging_arne as messaging_arne

def plannerd_thread(sm=None, pm=None, arne_sm=None):

  config_rt_process(2, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  PL = Planner(CP)
  PP = PathPlanner(CP)

  VM = VehicleModel(CP)

  if sm is None:
    sm = messaging.SubMaster(['carState', 'controlsState', 'radarState', 'model', 'liveParameters', 'liveMapData'],
                             poll=['radarState', 'model'])
  if arne_sm is None:
    arne_sm = messaging_arne.SubMaster(['arne182Status', 'latControl', 'modelLongButton'])
  if pm is None:
    pm = messaging.PubMaster(['plan', 'liveLongitudinalMpc', 'pathPlan', 'liveMpc'])

  sm['liveParameters'].valid = True
  sm['liveParameters'].sensorValid = True
  sm['liveParameters'].steerRatio = CP.steerRatio
  sm['liveParameters'].stiffnessFactor = 1.0

  while True:
    sm.update()
    arne_sm.update(0)

    if sm.updated['model']:
      PP.update(sm, pm, CP, VM)
    if sm.updated['radarState']:
      PL.update(sm, pm, CP, VM, PP, arne_sm)


def main(sm=None, pm=None, arne_sm=None):
  plannerd_thread(sm, pm, arne_sm)


if __name__ == "__main__":
  main()
