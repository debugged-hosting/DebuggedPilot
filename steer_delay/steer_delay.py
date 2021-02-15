#!/usr/bin/env python3
import warnings

warnings.simplefilter(action='ignore', category=FutureWarning)  # for seaborn
import os
import pickle
import sys
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm  # type: ignore
from selfdrive.car.toyota.values import STEER_THRESHOLD
from scipy.signal import correlate
import seaborn as sns
from selfdrive.config import Conversions as CV

from common.realtime import DT_CTRL
from tools.lib.logreader import MultiLogIterator
from opendbc.can.parser import CANParser


MIN_SAMPLES = int(5 / DT_CTRL)
MAX_SAMPLES = int(15 / DT_CTRL)


def to_signed(n, bits):
  if n >= (1 << max((bits - 1), 0)):
    n = n - (1 << max(bits, 0))
  return n


def find_steer_delay(cp, plot=False):
  BASEDIR = '/data/openpilot'
  use_dir = '{}/steer_delay/rlogs/good'.format(BASEDIR)  # change to a directory of rlogs
  files = os.listdir(use_dir)
  files = [f for f in files if '.ini' not in f]

  routes = [[files[0]]]  # this mess ensures we process each route's segments independantly since sorting will join samples from random routes
  for rt in files[1:]:  # todo: clean up
    rt_name = ''.join(rt.split('--')[:2])
    if rt_name != ''.join(routes[-1][-1].split('--')[:2]):
      routes.append([rt])
    else:
      routes[-1].append(rt)

  lrs = []
  for _routes in routes:
    lrs.append(MultiLogIterator([os.path.join(use_dir, i) for i in _routes], wraparound=False))

  data = [[]]
  for lr in lrs:
    engaged, steering_pressed = False, False
    torque_cmd, steer_angle = None, None
    yaw_rate = None
    v_ego = None

    all_msgs = sorted(lr, key=lambda msg: msg.logMonoTime)
    del lr

    for msg in tqdm(all_msgs):
      if msg.which() == 'liveLocationKalman':
        yaw_rate = msg.liveLocationKalman.angularVelocityCalibrated.value[2] * -150
      elif msg.which() == 'carState':
        v_ego = msg.carState.vEgo

      if msg.which() not in ['can', 'sendcan']:
        continue

      updated = cp.update_string(msg.as_builder().to_bytes())
      for u in updated:
        if u == 0x2e4:  # STEERING_LKA
          engaged = bool(cp.vl[u]['STEER_REQUEST'])
          torque_cmd = cp.vl[u]['STEER_TORQUE_CMD']
        elif u == 0x260:  # STEER_TORQUE_SENSOR
          steering_pressed = abs(cp.vl[u]['STEER_TORQUE_DRIVER']) > STEER_THRESHOLD
        elif u == 0x25:  # STEER_ANGLE_SENSOR
          steer_angle = cp.vl[u]['STEER_ANGLE'] + cp.vl[u]['STEER_FRACTION']

      if (engaged and not steering_pressed and None not in [torque_cmd, steer_angle, yaw_rate, v_ego] and  # creates uninterupted sections of engaged data
              v_ego > 5 * CV.MPH_TO_MS):  # todo: experiment with 5 or lower
        data[-1].append({'engaged': engaged, 'torque_cmd': torque_cmd, 'steering_pressed': steering_pressed,
                         'steer_angle': steer_angle, 'time': msg.logMonoTime * 1e-9, 'yaw_rate': yaw_rate, 'v_ego': v_ego})
      elif len(data[-1]):
        data.append([])

  del all_msgs

  split = [sec for sec in data if MAX_SAMPLES > len(sec) > MIN_SAMPLES]  # long enough sections
  del data
  assert len(split) > 0, "Not enough valid sections of samples"

  delays = []
  ptps = []
  seq_lens = []
  new_split = []
  mean_vels = []
  for idx, data in enumerate(split):
    torque = np.array([line['torque_cmd'] for line in data])
    angles = np.array([line['steer_angle'] for line in data])
    if angles.std() == 0 or torque.std() == 0:
      print('warning: angles or torque std is 0! skipping...')
      continue
    angles_ptp = abs(angles.ptp())  # todo: abs shouldn't be needed, but just in case
    if angles_ptp <= 5:
      print('angle range too low ({} <= 5)! skipping...'.format(angles_ptp))
      continue
    print('peak to peak: {}'.format(angles_ptp))

    # diff = max(torque) / max(angles)  # todo: not sure which regularization method is better. normalization
    # angles *= diff

    Y1 = (angles - angles.mean()) / angles.std()  # todo: or standardization
    Y2 = (torque - torque.mean()) / torque.std()

    plt.clf()
    plt.title('before offset')
    plt.plot(angles, label='angle')
    plt.plot(torque / (max(torque) / max(angles)), label='torque')
    plt.legend()
    plt.savefig('{}/steer_delay/plots/{}__before.png'.format(BASEDIR, idx))

    # xcorr = correlate(Y1, Y2)[angles.size - 1:]  # indexing forces positive offset (torque always precedes angle)
    xcorr = correlate(Y1, Y2)  # indexing forces positive offset (torque always precedes angle)
    time_shift = np.arange(1 - angles.size, angles.size)[xcorr.argmax()]

    if 0 < time_shift < 100:  # still plot them to debug todo: remove them
      delays.append(time_shift)
      ptps.append(angles_ptp)
      seq_lens.append(len(data))
      new_split.append(split)
      mean_vels.append(np.mean([line['v_ego'] for line in data]))

    print('len: {}'.format(len(data)))
    print('time shift: {}'.format(time_shift))
    print()

    torque = np.roll(torque, time_shift)
    plt.clf()
    plt.title('after offset ({})'.format(time_shift))
    plt.plot(angles, label='angle')
    plt.plot(torque / (max(torque) / max(angles)), label='torque')
    plt.legend()
    plt.savefig('{}/steer_delay/plots/{}_after.png'.format(BASEDIR, idx))

  plt.clf()
  sns.distplot(delays, bins=40, label='delays')
  plt.legend()
  plt.savefig('{}/steer_delay/dist.png'.format(BASEDIR))

  with open('{}/steer_delay/data'.format(BASEDIR), 'wb') as f:
    pickle.dump(new_split, f)

  print('mean vels:  {}'.format(np.round(np.array(mean_vels) * CV.MS_TO_MPH, 2).tolist()))  # todo: add vel list
  print('seq lens:   {}'.format(seq_lens))
  print('ptp angles: {}'.format(ptps))
  print('delays:     {}'.format(delays))

  print('\nmedian: {}'.format(np.median(delays)))
  print('mean: {}'.format(np.mean(delays)))


if __name__ == "__main__":
  # r = Route("14431dbeedbf3558%7C2020-11-10--22-24-34")
  # lr = MultiLogIterator(r.log_paths(), wraparound=False)

  signals = [
    ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
    ("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),
    ("STEER_RATE", "STEER_ANGLE_SENSOR", 0),
    ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
    ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
    ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
    ("STEER_REQUEST", "STEERING_LKA", 0),
    ("STEER_TORQUE_CMD", "STEERING_LKA", 0),
    ("YAW_RATE", "KINEMATICS", 0)
  ]
  cp = CANParser("toyota_corolla_2017_pt_generated", signals)


  find_steer_delay(cp, plot="--plot" in sys.argv)
