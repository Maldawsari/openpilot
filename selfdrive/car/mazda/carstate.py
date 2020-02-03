
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from opendbc.can.parser import CANParser
from selfdrive.car.mazda.values import DBC, LKAS_LIMITS

def get_powertrain_can_parser(CP, canbus):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("LEFT_BLINK", "BLINK_INFO", 0),
    ("RIGHT_BLINK", "BLINK_INFO", 0),
    ("STEER_ANGLE", "STEER", 0),
    ("STEER_ANGLE_RATE", "STEER_RATE", 0),
    ("LKAS_BLOCK", "STEER_RATE", 0),
    ("LKAS_TRACK_STATE", "STEER_RATE", 0),
    ("HANDS_OFF_5_SECONDS", "STEER_RATE", 0),
    ("STEER_TORQUE_SENSOR", "STEER_TORQUE", 0),
    ("STEER_TORQUE_MOTOR", "STEER_TORQUE", 0),
    ("FL", "WHEEL_SPEEDS", 0),
    ("FR", "WHEEL_SPEEDS", 0),
    ("RL", "WHEEL_SPEEDS", 0),
    ("RR", "WHEEL_SPEEDS", 0),
    ("CRZ_ACTIVE", "CRZ_CTRL", 0),
    ("STANDSTILL","PEDALS", 0),
    ("BRAKE_ON","PEDALS", 0),
    ("GEAR","GEAR", 0),
    ("DRIVER_SEATBELT", "SEATBELT", 0),
    ("FL", "DOORS", 0),
    ("FR", "DOORS", 0),
    ("BL", "DOORS", 0),
    ("BR", "DOORS", 0),
    ("PEDAL_GAS", "ENGINE_DATA", 0),
    ("RES", "CRZ_BTNS", 0),
    ("SET_P", "CRZ_BTNS", 0),
    ("SET_M", "CRZ_BTNS", 0),
  ]

  checks = [
    # sig_address, frequency
    ("BLINK_INFO", 10),
    ("STEER", 67),
    ("STEER_RATE", 83),
    ("STEER_TORQUE", 83),
    ("WHEEL_SPEEDS", 100),
    ("ENGINE_DATA", 100),
    ("CRZ_CTRL", 50),
    ("CRZ_BTNS", 10),
    ("PEDALS", 50),
    ("SEATBELT", 10),
    ("DOORS", 10),
    ("GEAR", 20),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.powertrain)

def get_cam_can_parser(CP, canbus):
  signals = [
    # sig_name, sig_address, default

    ("LKAS_REQUEST",     "CAM_LKAS", 0),
    ("CTR",              "CAM_LKAS", 0),
    ("ERR_BIT_1",        "CAM_LKAS", 0),
    ("LDW",              "CAM_LKAS", 0),
    ("LINE_NOT_VISIBLE", "CAM_LKAS", 0),
    ("BIT_1",            "CAM_LKAS", 0),
    ("ERR_BIT_2",        "CAM_LKAS", 0),
    ("BIT_2",            "CAM_LKAS", 0),
    ("CHKSUM",           "CAM_LKAS", 0),
  ]

  checks = [
    # sig_address, frequency
    ("CAM_LKAS",      16),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.cam)

class STEER_LKAS():
  def __init__(self):
    self.block = 1
    self.track = 1
    self.handsoff = 0

class CarState():
  def __init__(self, CP, canbus):
    # initialize can parser
    self.CP = CP
    self.steer_lkas = STEER_LKAS()
    self.car_fingerprint = CP.carFingerprint
    self.blinker_on = False
    self.prev_blinker_on = False
    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False
    self.steer_torque_driver = 0
    self.steer_not_allowed = False
    self.main_on = False
    self.acc_active_last = False
    self.speed_kph = 0
    self.lkas_speed_lock = False
    self.low_speed_lockout = True

    # vEgo kalman filter
    dt = 0.01

    self.v_ego_kf = KF1D(x0=[[0.], [0.]],
                         A=[[1., dt], [0.0, 1.]],
                         C=[1., 0.],
                         K=[[0.12287673], [0.29666309]])

    self.v_ego = 0.

  def update(self, pt_cp, cam_cp):

    self.v_wheel_fl = pt_cp.vl["WHEEL_SPEEDS"]['FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = pt_cp.vl["WHEEL_SPEEDS"]['FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = pt_cp.vl["WHEEL_SPEEDS"]['RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = pt_cp.vl["WHEEL_SPEEDS"]['RR'] * CV.KPH_TO_MS
    v_wheel = (self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4.

    self.v_ego_raw = v_wheel
    self.speed_kph = v_wheel // CV.KPH_TO_MS

    v_ego_x = self.v_ego_kf.update(v_wheel)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.prev_blinker_on = self.blinker_on
    self.left_blinker_on = pt_cp.vl["BLINK_INFO"]['LEFT_BLINK'] == 1
    self.right_blinker_on = pt_cp.vl["BLINK_INFO"]['RIGHT_BLINK'] == 1
    self.blinker_on = self.left_blinker_on or self.right_blinker_on

    self.steer_torque_driver = pt_cp.vl["STEER_TORQUE"]['STEER_TORQUE_SENSOR']
    self.steer_torque_motor = pt_cp.vl["STEER_TORQUE"]['STEER_TORQUE_MOTOR']

    self.steer_override = abs(self.steer_torque_driver) > LKAS_LIMITS.STEER_THRESHOLD

    self.angle_steers = pt_cp.vl["STEER"]['STEER_ANGLE']
    self.angle_steers_rate = pt_cp.vl["STEER_RATE"]['STEER_ANGLE_RATE']

    #self.standstill = pt_cp.vl["PEDALS"]['STANDSTILL'] == 1
    #self.brake_pressed = pt_cp.vl["PEDALS"]['BREAK_PEDAL_1'] == 1

    self.standstill = self.v_ego_raw < 0.01

    self.door_open = any([pt_cp.vl["DOORS"]['FL'],
                          pt_cp.vl["DOORS"]['FR'],
                          pt_cp.vl["DOORS"]['BL'],
                          pt_cp.vl["DOORS"]['BR']])

    self.seatbelt_unlatched =  pt_cp.vl["SEATBELT"]['DRIVER_SEATBELT'] == 0

    self.user_gas_pressed = True if pt_cp.vl["ENGINE_DATA"]['PEDAL_GAS'] > 0.0001 else False

    # No steer if block signal is on
    self.steer_lkas.block = pt_cp.vl["STEER_RATE"]['LKAS_BLOCK']
    # track driver torque, on if torque is not detected
    self.steer_lkas.track = pt_cp.vl["STEER_RATE"]['LKAS_TRACK_STATE']
    # On if no driver torque the last 5 seconds
    self.steer_lkas.handsoff = pt_cp.vl["STEER_RATE"]['HANDS_OFF_5_SECONDS']

    # LKAS is enabled at 50kph going up and disabled at 45kph going down
    if self.speed_kph > LKAS_LIMITS.ENABLE_SPEED and self.low_speed_lockout:
      self.low_speed_lockout = False
    elif self.speed_kph < LKAS_LIMITS.DISABLE_SPEED and not self.low_speed_lockout:
      self.low_speed_lockout = True

    if (self.low_speed_lockout or self.steer_lkas.block) and self.speed_kph < LKAS_LIMITS.ENABLE_SPEED:
        if not self.lkas_speed_lock:
          self.lkas_speed_lock = True
    elif self.lkas_speed_lock:
      self.lkas_speed_lock = False

    # if any of the cruize buttons is pressed force state update
    if not self.low_speed_lockout and any([pt_cp.vl["CRZ_BTNS"]['RES'],
                                           pt_cp.vl["CRZ_BTNS"]['SET_P'],
                                           pt_cp.vl["CRZ_BTNS"]['SET_M']]):
      self.acc_active = True
      self.v_cruise_pcm =  self.v_ego_raw

    self.main_on = pt_cp.vl["CRZ_CTRL"]['CRZ_ACTIVE']
    if not self.main_on:
      self.acc_active = False

    if self.acc_active != self.acc_active_last:
      self.v_cruise_pcm =  self.speed_kph
      self.acc_active_last = self.acc_active

    self.steer_error = False
    self.brake_error = False

    #self.steer_not_allowed = self.steer_lkas.block == 1

    self.cam_lkas = cam_cp.vl["CAM_LKAS"]
