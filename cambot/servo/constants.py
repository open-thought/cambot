"""Feetech STS3215 register addresses, motor configuration, and conversions.

Single source of truth for all servo constants used across the codebase.
"""

import math

# --- Protocol ---
PROTOCOL_VERSION = 0  # STS series
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 1_000_000

# --- Position encoding ---
STEPS_PER_REV = 4096
STEPS_TO_RAD = 2.0 * math.pi / STEPS_PER_REV
RAD_TO_STEPS = STEPS_PER_REV / (2.0 * math.pi)
STEPS_TO_DEG = 360.0 / STEPS_PER_REV
POS_MIN = 0
POS_MAX = 4095

# Sign-magnitude bit positions
POS_SIGN_BIT = 15       # position register (addr 42, 56)
OFFSET_SIGN_BIT = 11    # homing offset register (addr 31)
LOAD_SIGN_BIT = 10      # present load register (addr 60)
OFFSET_MAX = (1 << OFFSET_SIGN_BIT) - 1  # 2047

# --- EPROM register addresses ---
ADDR_FIRMWARE_MAJOR = 0
ADDR_FIRMWARE_MINOR = 1
ADDR_MODEL_NUMBER = 3
ADDR_ID = 5              # DANGER: do not bulk-write
ADDR_BAUD_RATE = 6
ADDR_RETURN_DELAY_TIME = 7
ADDR_RESPONSE_STATUS_LEVEL = 8
ADDR_MIN_POSITION_LIMIT = 9
ADDR_MAX_POSITION_LIMIT = 11
ADDR_MAX_TEMPERATURE_LIMIT = 13
ADDR_MAX_VOLTAGE_LIMIT = 14
ADDR_MIN_VOLTAGE_LIMIT = 15
ADDR_MAX_TORQUE_LIMIT = 16
ADDR_PHASE = 18
ADDR_UNLOADING_CONDITION = 19
ADDR_LED_ALARM_CONDITION = 20
ADDR_P_COEFFICIENT = 21
ADDR_D_COEFFICIENT = 22
ADDR_I_COEFFICIENT = 23
ADDR_MINIMUM_STARTUP_FORCE = 24
ADDR_CW_DEAD_ZONE = 26
ADDR_CCW_DEAD_ZONE = 27
ADDR_PROTECTION_CURRENT = 28
ADDR_ANGULAR_RESOLUTION = 30
ADDR_HOMING_OFFSET = 31
ADDR_OPERATING_MODE = 33
ADDR_PROTECTIVE_TORQUE = 34
ADDR_PROTECTION_TIME = 35
ADDR_OVERLOAD_TORQUE = 36
ADDR_VEL_P_COEFF = 37
ADDR_OVER_CURRENT_PROT_TIME = 38
ADDR_VEL_I_COEFF = 39

# --- SRAM register addresses ---
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_TIME = 44
ADDR_GOAL_VELOCITY = 46
ADDR_TORQUE_LIMIT = 48
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VELOCITY = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_STATUS = 65
ADDR_MOVING = 66
ADDR_PRESENT_CURRENT = 69

# --- Factory register addresses ---
ADDR_MOVING_VELOCITY_THRESHOLD = 80
ADDR_DTS = 81
ADDR_VELOCITY_UNIT_FACTOR = 82
ADDR_HTS = 83
ADDR_MAX_VELOCITY_LIMIT = 84
ADDR_MAXIMUM_ACCELERATION = 85
ADDR_ACCELERATION_MULTIPLIER = 86

# --- SRAM defaults ---
DEFAULT_ACCELERATION = 254
DEFAULT_TORQUE_LIMIT = 900

# --- PID factory defaults ---
PID_FACTORY_DEFAULTS = (16, 32, 0)  # (P, D, I)

# --- Motor configuration ---
# Canonical URDF joint names (order matches motor IDs 1-6)
JOINT_NAMES = [
    "base_yaw", "shoulder_pitch", "elbow_pitch",
    "wrist_pitch", "wrist_yaw", "camera_roll",
]

# Motor ID -> joint name
MOTOR_IDS = {name: mid for mid, name in enumerate(JOINT_NAMES, start=1)}

# Joint name -> motor ID (reverse of MOTOR_IDS)
MOTOR_NAMES = {mid: name for name, mid in MOTOR_IDS.items()}

# Legacy SO-101 names for display compatibility
LEGACY_MOTOR_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

# --- Register table for read_all_params ---
# (name, address, size, description, decode_key_or_None)
# decode_key: "sm15", "sm11", "sm10" for sign-magnitude decoding
EPROM_REGISTERS = [
    ("Firmware_Major", 0, 1, "Firmware major version", None),
    ("Firmware_Minor", 1, 1, "Firmware minor version", None),
    ("Model_Number", 3, 2, "Model number (777=STS3215)", None),
    ("ID", 5, 1, "Motor ID", None),
    ("Baud_Rate", 6, 1, "Baud rate index (0=1M)", None),
    ("Return_Delay_Time", 7, 1, "Response delay (x2 us)", None),
    ("Response_Status_Level", 8, 1, "Status return level", None),
    ("Min_Position_Limit", 9, 2, "Min position limit", None),
    ("Max_Position_Limit", 11, 2, "Max position limit", None),
    ("Max_Temperature_Limit", 13, 1, "Max temp limit (C)", None),
    ("Max_Voltage_Limit", 14, 1, "Max voltage limit (x0.1V)", None),
    ("Min_Voltage_Limit", 15, 1, "Min voltage limit (x0.1V)", None),
    ("Max_Torque_Limit", 16, 2, "Max torque (0-1000 = 0-100%)", None),
    ("Phase", 18, 1, "Phase", None),
    ("Unloading_Condition", 19, 1, "Unloading condition", None),
    ("LED_Alarm_Condition", 20, 1, "LED alarm condition", None),
    ("P_Coefficient", 21, 1, "PID P gain", None),
    ("D_Coefficient", 22, 1, "PID D gain", None),
    ("I_Coefficient", 23, 1, "PID I gain", None),
    ("Minimum_Startup_Force", 24, 2, "Min startup force", None),
    ("CW_Dead_Zone", 26, 1, "CW dead zone", None),
    ("CCW_Dead_Zone", 27, 1, "CCW dead zone", None),
    ("Protection_Current", 28, 2, "Protection current (mA)", None),
    ("Angular_Resolution", 30, 1, "Angular resolution", None),
    ("Homing_Offset", 31, 2, "Homing offset (sign-mag bit11)", "sm11"),
    ("Operating_Mode", 33, 1, "Mode (0=pos, 1=speed, 3=step)", None),
    ("Protective_Torque", 34, 1, "Protective torque (%)", None),
    ("Protection_Time", 35, 1, "Protection time", None),
    ("Overload_Torque", 36, 1, "Overload torque (%)", None),
    ("Vel_P_Coeff", 37, 1, "Velocity P coefficient", None),
    ("Over_Current_Prot_Time", 38, 1, "Over-current protection time", None),
    ("Vel_I_Coeff", 39, 1, "Velocity I coefficient", None),
]

SRAM_REGISTERS = [
    ("Torque_Enable", 40, 1, "Torque enabled", None),
    ("Acceleration", 41, 1, "Acceleration", None),
    ("Goal_Position", 42, 2, "Goal position (sign-mag bit15)", "sm15"),
    ("Goal_Time", 44, 2, "Goal time (ms)", None),
    ("Goal_Velocity", 46, 2, "Goal velocity (sign-mag bit15)", "sm15"),
    ("Torque_Limit", 48, 2, "Runtime torque limit (0-1000)", None),
    ("Lock", 55, 1, "EPROM lock (1=locked)", None),
    ("Present_Position", 56, 2, "Present position (sign-mag bit15)", "sm15"),
    ("Present_Velocity", 58, 2, "Present velocity (sign-mag bit15)", "sm15"),
    ("Present_Load", 60, 2, "Present load (sign-mag bit10)", "sm10"),
    ("Present_Voltage", 62, 1, "Voltage (x0.1V)", None),
    ("Present_Temperature", 63, 1, "Temperature (C)", None),
    ("Status", 65, 1, "Status flags", None),
    ("Moving", 66, 1, "Is moving", None),
    ("Present_Current", 69, 2, "Current (mA)", None),
]

FACTORY_REGISTERS = [
    ("Moving_Velocity_Threshold", 80, 1, "Moving velocity threshold", None),
    ("DTs", 81, 1, "DTs (ms)", None),
    ("Velocity_Unit_Factor", 82, 1, "Velocity unit factor", None),
    ("Hts", 83, 1, "Hts (ns)", None),
    ("Max_Velocity_Limit", 84, 1, "Max velocity limit", None),
    ("Maximum_Acceleration", 85, 1, "Max acceleration", None),
    ("Acceleration_Multiplier", 86, 1, "Accel multiplier (when accel=0)", None),
]

ALL_REGISTERS = EPROM_REGISTERS + SRAM_REGISTERS + FACTORY_REGISTERS
REGISTER_BY_ADDR = {addr: (name, size) for name, addr, size, _, _ in ALL_REGISTERS}
EPROM_ADDRS = {addr for _, addr, _, _, _ in EPROM_REGISTERS}
