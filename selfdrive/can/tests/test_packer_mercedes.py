import unittest
import random

from selfdrive.can.tests.packer_old import CANPacker as CANPackerOld
from selfdrive.can.packer import CANPacker
import selfdrive.car.mercedes.mercedescan as mercedescan
from selfdrive.car.mercedes.values import CAR as mercedes_car


class TestPackerMethods(unittest.TestCase):
  def setUp(self):
    self.mercedes_cp_old = CANPackerOld("mercedes_E350_2011_pre_final")
    self.mercedes_cp = CANPacker("mercedes_E350_2011_pre_final")

  def test_correctness(self):
    # Test all cars' commands, randomize the params.
    for _ in range(1000):
      apply_steer = (random.randint(0, 2) % 2 == 0)
      frame = random.randint(1, 65536)
      steer_step = random.randint(1, 65536)
      m_old = mercedescan.create_steering_control(self.mercedes_cp_old, mercedes_car.ECLASS, apply_steer, frame, steer_step)
      m = mercedescan.create_steering_control(self.mercedes_cp, mercedes_car.ECLASS, apply_steer, frame, steer_step)
      self.assertEqual(m_old, m)

      m_old = mercedescan.create_steering_status(self.mercedes_cp_old, mercedes_car.ECLASS, apply_steer, frame, steer_step)
      m = mercedescan.create_steering_status(self.mercedes_cp, mercedes_car.ECLASS, apply_steer, frame, steer_step)
      self.assertEqual(m_old, m)

      es_distance_msg = {}
      pcm_cancel_cmd = (random.randint(0, 2) % 2 == 0)
      m_old = mercedescan.create_es_distance(self.mercedes_cp_old, es_distance_msg, pcm_cancel_cmd)
      m = mercedescan.create_es_distance(self.mercedes_cp, es_distance_msg, pcm_cancel_cmd)
      self.assertEqual(m_old, m)


if __name__ == "__main__":
  unittest.main()
