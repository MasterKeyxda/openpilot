import copy
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.can.parser import CANParser
from selfdrive.car.mercedes.values import DBC, STEER_THRESHOLD

def get_powertrain_can_parser(CP):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("STEER_TORQUE", "STEERING_1", 0),
    ("STEER_ANGLE", "STEERING_2", 0),
    ("STEER_RATE", "STEERING_2", 0),
    ("CC_CANCEL", "DRIVER_CONTROL", 0),
    ("CC_RESUME", "DRIVER_CONTROL", 0),
    ("CC_5MPH_ACCEL", "DRIVER_CONTROL", 0),
    ("CC_5MPH_DECEL", "DRIVER_CONTROL", 0),
    ("CC_1MPH_ACCEL", "DRIVER_CONTROL", 0),
    ("CC_1MPH_DECEL", "DRIVER_CONTROL", 0),
    ("BRAKE_PRESSED", "BRAKE_2", 0),
    ("DRIVER_BRAKE", "BRAKE_2", 0),
    ("BRAKE_POSITION", "BRAKE_2", 0),
    ("THROTTLE_POSITION", "THROTTLE_1_RPM", 0),
    ("BLINKER_LEFT", "DRIVER_CONTROL", 0),
    ("BLINKER_RIGHT", "DRIVER_CONTROL", 0),
    ("DRIVER_BELT", "SEATBELT", 0),
    ("WHEEL_SPEED_FL", "WHEEL_SPEED", 0),
    ("WHEEL_SPEED_FR", "WHEEL_SPEED", 0),
    ("WHEEL_SPEED_RL", "WHEEL_SPEED", 0),
    ("WHEEL_SPEED_RR", "WHEEL_SPEED", 0),
    ("DRIVER_DOOR", "DOORS", 1),
    ("PASSENGER_DOOR", "DOORS", 1),
    ("REAR_PASSENGER_DRIVER", "DOORS", 1),
    ("REAR_PASSENGER_PASSENGER", "DOORS", 1),
    ("CRUISE_SET_UP_DOWN", "CRUISE_CONTROL_3", 0),
    ("CRUISE_ON", "CRUISE_CONTROL", 0)
  ]

  checks = [
    # sig_address, frequency:
    ("DOORS", 1),
  ]
  #print ((DBC[CP.carFingerprint]['pt']))
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)


def get_camera_can_parser(CP):
  signals = []

  checks = []

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)


class CarState():
  def __init__(self, CP):
    # initialize can parser
    self.CP = CP

    self.car_fingerprint = CP.carFingerprint
    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False
    self.steer_torque_driver = 0
    self.steer_not_allowed = False
    self.main_on = False

    # vEgo kalman filter
    dt = 0.01
    self.v_ego_kf = KF1D(x0=[[0.], [0.]],
                         A=[[1., dt], [0., 1.]],
                         C=[1., 0.],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.

  def update(self, cp, cp_cam):
    self.pedal_gas = cp.vl["THROTTLE_1_RPM"]['THROTTLE_POSITION']
    self.brake_pressure = cp.vl["BRAKE_2"]['BRAKE_POSITION']
    self.user_gas_pressed = self.pedal_gas > 0
    self.brake_pressed = self.brake_pressure > 0
    self.brake_lights = bool(self.brake_pressed)

    self.v_wheel_fl = cp.vl["WHEEL_SPEED"]['WHEEL_SPEED_FL'] * CV.MPH_TO_MS
    self.v_wheel_fr = cp.vl["WHEEL_SPEED"]['WHEEL_SPEED_FR'] * CV.MPH_TO_MS
    self.v_wheel_rl = cp.vl["WHEEL_SPEED"]['WHEEL_SPEED_RL'] * CV.MPH_TO_MS
    self.v_wheel_rr = cp.vl["WHEEL_SPEED"]['WHEEL_SPEED_RR'] * CV.MPH_TO_MS

    self.v_cruise_pcm = cp.vl["CRUISE_CONTROL_3"]['CRUISE_SET_UP_DOWN'] #orginally ["ES_DashStatus"]['Cruise_Set_Speed'] from sub
    
    self.v_cruise_pcm *= CV.MPH_TO_KPH
    # 1 = imperial, 6 = metric
    #if cp.vl["Dash_State"]['Units'] == 1:
    #  self.v_cruise_pcm *= CV.MPH_TO_KPH

    v_wheel = ((self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4)
    # Kalman filter, even though Hyundai raw wheel speed is heaviliy filtered by default
    if abs(v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_wheel], [0.0]]

    self.v_ego_raw = v_wheel
    v_ego_x = self.v_ego_kf.update(v_wheel)

    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = self.v_ego_raw < 0.01

    print('Cruise Control')
    print(cp.vl["CRUISE_CONTROL"]['CRUISE_ON'])
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.left_blinker_on = cp.vl["DRIVER_CONTROL"]['BLINKER_LEFT'] == 1
    self.right_blinker_on = cp.vl["DRIVER_CONTROL"]['BLINKER_RIGHT'] == 1
    self.seatbelt_unlatched = bool(cp.vl["SEATBELT"]['DRIVER_BELT']) #true if the bit value is 1
    self.steer_torque_driver = cp.vl["STEERING_1"]['STEER_TORQUE']
    self.acc_active = cp.vl["CRUISE_CONTROL"]['CRUISE_ON'] # Check with the bit value, true if 1
    self.main_on = cp.vl["CRUISE_CONTROL"]['CRUISE_ON'] # Check with the bit value, true if 1
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.car_fingerprint]
    self.angle_steers = cp.vl["STEERING_2"]['STEER_ANGLE']
    self.door_open = not all([cp.vl["DOORS"]['REAR_PASSENGER_DRIVER'],
      cp.vl["DOORS"]['REAR_PASSENGER_PASSENGER'],
      cp.vl["DOORS"]['PASSENGER_DOOR'],
      cp.vl["DOORS"]['DRIVER_DOOR']])

    #self.es_distance_msg = copy.copy(cp_cam.vl["ES_Distance"])
    #self.es_lkas_msg = copy.copy(cp_cam.vl["ES_LKAS_State"])
    #self.turn_blinker_on = copy.copy(cp.vl["DRIVER_CONTROL"]) #Added by keyur
