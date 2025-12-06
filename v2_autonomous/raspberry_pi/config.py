"""
v2 自走車控制參數配置

調整此檔案的參數後，重新啟動 controller.py 即可生效
無需重新上傳 Arduino 程式
"""

# ==================== 串口設定 ====================
SERIAL_PORT = '/dev/ttyACM0'  # Arduino 串口 (可能是 ttyACM0 或 ttyACM1)
SERIAL_BAUDRATE = 115200

# ==================== 距離閾值 (cm) ====================
TARGET_RIGHT_DIST = 15      # 目標右側距離
FRONT_STOP_DIST = 20        # 前方停止/角落偵測距離
FRONT_SLOW_DIST = 40        # 前方減速距離
CORNER_RIGHT_DIST = 30      # 角落右側閾值

# ==================== 速度參數 (-1.0 ~ +1.0) ====================
BASE_LINEAR_SPEED = 0.20    # 基礎前進速度 (降低以避免急停急走)
BACKUP_SPEED = 0.0          # 後退速度 (已停用，角落直接轉彎)
TURN_ANGULAR_SPEED = 0.25   # 角落轉彎角速度 (降低以平滑轉彎)
FIND_WALL_LINEAR = 0.20     # 尋牆前進速度 (同步降低)

# ==================== 左右輪速度修正 ====================
# 用於補償左右輪馬達差異，使直行時不偏移
# 1.0 = 正常速度，<1.0 = 減速，>1.0 = 加速
# 左>右 = 車子會右偏 (朝牆壁靠近)
LEFT_MOTOR_SCALE = 1.0      # 左輪速度倍率
RIGHT_MOTOR_SCALE = 0.95    # 右輪速度倍率 (尋牆時輕微右偏)

# ==================== PID 參數 ====================
DEFAULT_KP = 0.025          # 比例增益
DEFAULT_KI = 0.002          # 積分增益
DEFAULT_KD = 0.010          # 微分增益

# ==================== PWM 參數 ====================
MIN_EFFECTIVE_PWM = 45      # 死區 (低於此值馬達不轉)

# ==================== 角度參數 (度) ====================
CORNER_TURN_ANGLE = 85      # 角落轉彎角度
RED_AVOID_ANGLE = 45        # 紅色迴避角度

# ==================== 時間參數 (ms) ====================
BACKUP_DURATION_MS = 300    # 後退持續時間
TURN_TIMEOUT_MS = 3000      # 轉彎超時

# ==================== 紅色偵測參數 ====================
RED_AREA_THRESHOLD = 2000   # 紅色面積閾值
RED_CHECK_INTERVAL = 0.5    # 紅色檢查間隔（秒）

# 紅色迴避策略
# "vacuum_off" - 遇到紅色只關吸塵器，繼續沿牆 (簡單方案)
# "turn_avoid" - 遇到紅色左轉迴避 (原方案)
RED_AVOID_STRATEGY = "vacuum_off"

# 紅色消失後延遲重開吸塵器的時間 (秒)
# 避免吸到紅色區域邊緣
RED_CLEAR_DELAY = 1.5


def get_all_params() -> dict:
    """
    取得所有控制參數

    Returns:
        dict: 包含所有參數的字典
    """
    return {
        # 距離閾值
        'target_right_dist': TARGET_RIGHT_DIST,
        'front_stop_dist': FRONT_STOP_DIST,
        'front_slow_dist': FRONT_SLOW_DIST,
        'corner_right_dist': CORNER_RIGHT_DIST,
        # 速度參數
        'base_linear_speed': BASE_LINEAR_SPEED,
        'backup_speed': BACKUP_SPEED,
        'turn_angular_speed': TURN_ANGULAR_SPEED,
        'find_wall_linear': FIND_WALL_LINEAR,
        'left_motor_scale': LEFT_MOTOR_SCALE,
        'right_motor_scale': RIGHT_MOTOR_SCALE,
        # PID 參數
        'kp': DEFAULT_KP,
        'ki': DEFAULT_KI,
        'kd': DEFAULT_KD,
        # 其他參數
        'min_effective_pwm': MIN_EFFECTIVE_PWM,
        'corner_turn_angle': CORNER_TURN_ANGLE,
        'red_avoid_angle': RED_AVOID_ANGLE,
        'backup_duration_ms': BACKUP_DURATION_MS,
        'turn_timeout_ms': TURN_TIMEOUT_MS,
    }


if __name__ == "__main__":
    # 測試輸出
    print("=== v2 控制參數 ===")
    params = get_all_params()
    for key, value in params.items():
        print(f"  {key}: {value}")
