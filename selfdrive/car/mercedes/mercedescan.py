import copy
from cereal import car
from selfdrive.car.mercedes.values import CAR

VisualAlert = car.CarControl.HUDControl.VisualAlert

"""
def subaru_checksum(data):
  print(data)
  checksum = 0xFF
  temp_chk = 0
  bit_sum = 0

  for byte in data:
    shift = 0x80
    for i in range(8, 0, -1):
      bit_sum = byte & shift
      temp_chk = checksum & 0x80
      if bit_sum != 0:
        bit_sum = 0x1C
        if (temp_chk != 0):
          bit_sum = 1
        checksum = checksum << 1
        temp_chk = checksum | 1
        bit_sum ^= temp_chk
      else:
        if temp_chk != 0:
          bit_sum = 0x1D
        checksum = checksum << 1
        bit_sum ^= checksum
      checksum = bit_sum
      shift = shift >> 1
  return ~checksum & 0xFF

def subaru_checksum(packer, values, addr):
  dat = packer.make_can_msg(addr, 0, values)[2]
  print(dat)
  return (sum(dat[1:]) + (addr >> 8) + addr) & 0xff
"""

def subaru_checksum(packer, values, addr):
  #KEYUR EDITED FUNCTION
  data = packer.make_can_msg(addr, 0, values)[2]
  checksum = 0xFF
  temp_chk = 0
  bit_sum = 0

  for byte in data:
    shift = 0x80
    for i in range(8, 0, -1):
      bit_sum = byte & shift
      temp_chk = checksum & 0x80
      if bit_sum != 0:
        bit_sum = 0x1C
        if (temp_chk != 0):
          bit_sum = 1
        checksum = checksum << 1
        temp_chk = checksum | 1
        bit_sum ^= temp_chk
      else:
        if temp_chk != 0:
          bit_sum = 0x1D
        checksum = checksum << 1
        bit_sum ^= checksum
      checksum = bit_sum
      shift = shift >> 1
  return ~checksum & 0xFF

"""
def create_steering_control(packer, car_fingerprint, apply_steer, frame, steer_step):

  if car_fingerprint == CAR.ECLASS:
    #counts from 0 to 15 then back to 0 + 16 for enable bit
    print('--------------------HELP--------------------')
    idx = ((frame // steer_step) % 16)
    print(apply_steer)
    print(frame)
    print(steer_step)
    print(idx)
    values = {
      "Counter": idx,
      "LKAS_Output": apply_steer,
      "LKAS_Request": 1 if apply_steer != 0 else 0,
      "SET_1": 1
    }
    #values["Checksum"] = subaru_checksum(values)
    values["Checksum"] = subaru_checksum(packer, values, 0x122)
  return packer.make_can_msg("ES_LKAS", 0, values)
"""

def create_steering_control(packer, car_fingerprint, apply_steer, frame, steer_step):
  #KEYUR EDITED FUNCTION
  
  if car_fingerprint == CAR.ECLASS:
    #counts from 0 to 15 then back to 0 + 16 for enable bit
    idx = ((frame // steer_step) % 16)
    values = {
      "COUNTER": idx,
      "STEER_TORQUE": apply_steer,
    }
    #values["Checksum"] = subaru_checksum(values)
    values["CHECKSUM"] = subaru_checksum(packer, values, 0x0e)
  return packer.make_can_msg("STEERING_1", 0, values)
"""
def create_steering_status(packer, car_fingerprint, apply_steer, frame, steer_step):
  
  if car_fingerprint == CAR.ECLASS:
    values = {}
    #values["Checksum"] = subaru_checksum(0x322)
    values["Checksum"] = subaru_checksum(packer, values, 0x322)

  return packer.make_can_msg("ES_LKAS_State", 0, values)

def create_es_distance(packer, es_distance_msg, pcm_cancel_cmd):
  
  values = copy.copy(es_distance_msg)
  if pcm_cancel_cmd:
    values["Main"] = 1

  #values["Checksum"] = subaru_checksum(values)
  values["Checksum"] = subaru_checksum(packer, values, 545)

  return packer.make_can_msg("ES_Distance", 0, values)

def create_es_lkas(packer, es_lkas_msg, visual_alert, left_line, right_line):
  
  values = copy.copy(es_lkas_msg)
  if visual_alert == VisualAlert.steerRequired:
    values["Keep_Hands_On_Wheel"] = 1

  values["LKAS_Left_Line_Visible"] = int(left_line)
  values["LKAS_Right_Line_Visible"] = int(right_line)

  #values["Checksum"] = subaru_checksum(values)
  values["Checksum"] = subaru_checksum(packer, values, 545)

  return packer.make_can_msg("ES_LKAS_State", 0, values)
"""