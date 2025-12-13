#!/usr/bin/env python3
"""
config.py - 機電小車系統設定檔
版本: 1.0
日期: 2025-10-31
"""

# ==================== Serial 通訊設定 ====================
SERIAL_PORT = '/dev/ttyACM1'      # Arduino USB Serial (改成 USB，原本是 /dev/serial0)
SERIAL_BAUDRATE = 9600            # 鮑率 (改成 9600 匹配 Arduino，原本是 57600)
SERIAL_TIMEOUT = 0.1              # 讀取逾時 (秒)
COMMAND_TIMEOUT = 0.2             # 指令逾時 (秒) - 超過此時間未收到指令則停止

# ==================== 搖桿設定 ====================
JOYSTICK_DEVICE_ID = 0            # pygame joystick 編號
JOYSTICK_DEADZONE = 0.1           # 搖桿死區 (0.0-1.0)

# 搖桿軸映射（依據實際遙控器調整）
# 雙搖桿模式：左搖桿控制前後，右搖桿控制左右
JOYSTICK_AXIS_LINEAR = 1         # 線性速度軸（前進/後退） - 左搖桿上下
JOYSTICK_AXIS_ANGULAR = 2        # 角速度軸（左轉/右轉） - 右搖桿左右
JOYSTICK_AXIS_INVERT_LINEAR = True   # Y 軸反轉（向上推為正值）
JOYSTICK_AXIS_INVERT_ANGULAR = True  # X 軸反轉（修正左右轉方向）

# 按鈕映射
JOYSTICK_BUTTON_VACUUM = 0       # 吸塵器開關按鈕（通常是 A/X 按鈕）是A
JOYSTICK_BUTTON_EMERGENCY_STOP = 1  # 緊急停止按鈕

# ==================== 馬達設定 ====================
MAX_PWM_VALUE = 255               # PWM 最大值
MIN_PWM_VALUE = -255              # PWM 最小值

# 馬達校準係數（若左右輪速度不一致，可調整此係數）
MOTOR_LEFT_SCALE = 1.0            # 左輪速度倍率
MOTOR_RIGHT_SCALE = 1.0           # 右輪速度倍率

# ==================== 控制迴圈設定 ====================
CONTROL_LOOP_FREQUENCY = 20       # Hz (主控制迴圈頻率) - 降低到 20Hz 避免淹沒 Arduino
CONTROL_LOOP_PERIOD = 1.0 / CONTROL_LOOP_FREQUENCY  # 秒

# ==================== Serial 封包格式 ====================
# Pi → Arduino 馬達指令封包
PACKET_MOTOR_HEADER = 0xAA
PACKET_MOTOR_FOOTER = 0x55
PACKET_MOTOR_SIZE = 8

# Arduino → Pi 感測器封包
PACKET_SENSOR_HEADER = 0xBB
PACKET_SENSOR_FOOTER = 0x66
PACKET_SENSOR_SIZE = 8

# ==================== 感測器設定 ====================
SENSOR_UPDATE_FREQUENCY = 10      # Hz (感測器資料更新頻率)
SENSOR_INVALID_VALUE = 999        # 無效距離值

# 超聲波距離範圍
ULTRASONIC_MIN_DISTANCE = 2       # cm
ULTRASONIC_MAX_DISTANCE = 400     # cm

# ==================== 除錯設定 ====================
DEBUG_MODE = False                # 除錯模式（顯示詳細訊息）
VERBOSE_SERIAL = False            # 顯示 Serial 封包原始資料
SHOW_SENSOR_DATA = True           # 顯示感測器資料

# ==================== 安全設定 ====================
EMERGENCY_STOP_TIMEOUT = 0.2      # 秒 - 超過此時間未收到指令則緊急停止
ENABLE_WATCHDOG = True            # 啟用看門狗（監控通訊狀態）

# ==================== 日誌設定 ====================
LOG_LEVEL = 'INFO'                # DEBUG / INFO / WARNING / ERROR
LOG_TO_FILE = False               # 是否寫入日誌檔案
LOG_FILE_PATH = '/tmp/robot_controller.log'
