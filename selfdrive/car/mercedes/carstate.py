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
    ("SEATBELT_FL", "Dashlights", 0),
    ("WHEEL_SPEED_FL", "WHEEL_SPEED", 0),
    ("WHEEL_SPEED_FR", "WHEEL_SPEED", 0),
    ("WHEEL_SPEED_RL", "WHEEL_SPEED", 0),
    ("WHEEL_SPEED_RR", "WHEEL_SPEED", 0),
    ("DRIVER_DOOR", "DOORS", 1),
    ("PASSENGER_DOOR", "DOORS", 1),
    ("REAR_PASSENGER_DRIVER", "DOORS", 1),
    ("REAR_PASSENGER_PASSENGER", "DOORS", 1),
    ("Units", "Dash_State", 1),
    ("CRUISE_ON", "CRUISE_CONTROL", 0)
  ]

  checks = [
    # sig_address, frequency
    ("Dashlights", 10),
    ("DRIVER_CONTROL", 20),
    ("WHEEL_SPEED", 50),
    ("STEERING_1", 50),
    ("DOORS", 10),
    ("CRUISE_CONTROL", 10),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)


def get_camera_can_parser(CP):
  signals = [
    ("Cruise_Set_Speed", "ES_DashStatus", 0),

    ("Counter", "ES_Distance", 0),
    ("Signal1", "ES_Distance", 0),
    ("Signal2", "ES_Distance", 0),
    ("Main", "ES_Distance", 0),
    ("Signal3", "ES_Distance", 0),

    ("Checksum", "ES_LKAS_State", 0),
    ("Counter", "ES_LKAS_State", 0),
    ("Keep_Hands_On_Wheel", "ES_LKAS_State", 0),
    ("Empty_Box", "ES_LKAS_State", 0),
    ("Signal1", "ES_LKAS_State", 0),
    ("LKAS_ACTIVE", "ES_LKAS_State", 0),
    ("Signal2", "ES_LKAS_State", 0),
    ("Backward_Speed_Limit_Menu", "ES_LKAS_State", 0),
    ("LKAS_ENABLE_3", "ES_LKAS_State", 0),
    ("Signal3", "ES_LKAS_State", 0),
    ("LKAS_ENABLE_2", "ES_LKAS_State", 0),
    ("Signal4", "ES_LKAS_State", 0),
    ("LKAS_Left_Line_Visible", "ES_LKAS_State", 0),
    ("Signal6", "ES_LKAS_State", 0),
    ("LKAS_Right_Line_Visible", "ES_LKAS_State", 0),
    ("Signal7", "ES_LKAS_State", 0),
    ("FCW_Cont_Beep", "ES_LKAS_State", 0),
    ("FCW_Repeated_Beep", "ES_LKAS_State", 0),
    ("Throttle_Management_Activated", "ES_LKAS_State", 0),
    ("Traffic_light_Ahead", "ES_LKAS_State", 0),
    ("Right_Depart", "ES_LKAS_State", 0),
    ("Signal5", "ES_LKAS_State", 0),

  ]

  checks = [
    ("ES_DashStatus", 10),
  ]

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

    self.pedal_gas = cp.vl["THROTTLE_1_RPM"]['THROTTLE_PEDAL_POSITION']
    self.brake_pressure = cp.vl["BRAKE_2"]['BRAKE_POSITION']
    self.user_gas_pressed = self.pedal_gas > 0
    self.brake_pressed = self.brake_pressure > 0
    self.brake_lights = bool(self.brake_pressed)

    self.v_wheel_fl = cp.vl["WHEEL_SPEED"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = cp.vl["WHEEL_SPEED"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = cp.vl["WHEEL_SPEED"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = cp.vl["WHEEL_SPEED"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS

    self.v_cruise_pcm = cp_cam.vl["ES_DashStatus"]['Cruise_Set_Speed']
    # 1 = imperial, 6 = metric
    #if cp.vl["Dash_State"]['Units'] == 1:
    #  self.v_cruise_pcm *= CV.MPH_TO_KPH

    v_wheel = (self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4.
    # Kalman filter, even though Hyundai raw wheel speed is heaviliy filtered by default
    if abs(v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_wheel], [0.0]]

    self.v_ego_raw = v_wheel
    v_ego_x = self.v_ego_kf.update(v_wheel)

    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = self.v_ego_raw < 0.01

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.left_blinker_on = cp.vl["DRIVER_CONTROL"]['BLINKER_LEFT'] == 1
    self.right_blinker_on = cp.vl["DRIVER_CONTROL"]['BLINKER_RIGHT'] == 1
    self.seatbelt_unlatched = cp.vl["Dashlights"]['SEATBELT_FL'] == 1
    self.steer_torque_driver = cp.vl["STEER_TORQUE"]['STEER_TORQUE']
    self.acc_active = cp.vl["CRUISE_CONTROL"]['CRUISE_ON']
    self.main_on = cp.vl["CRUISE_CONTROL"]['CRUISE_ON']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.car_fingerprint]
    self.angle_steers = cp.vl["STEERING_2"]['STEER_ANGLE']
    self.door_open = any([cp.vl["DOORS"]['REAR_PASSENGER_DRIVER'],
      cp.vl["DOORS"]['REAR_PASSENGER_PASSENGER'],
      cp.vl["DOORS"]['PASSENGER_DOOR'],
      cp.vl["DOORS"]['DRIVER_DOOR']])

    self.es_distance_msg = copy.copy(cp_cam.vl["ES_Distance"])
    self.es_lkas_msg = copy.copy(cp_cam.vl["ES_LKAS_State"])
