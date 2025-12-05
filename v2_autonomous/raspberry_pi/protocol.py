"""
Arduino-Pi Communication Protocol Definition

Architecture:
- Arduino handles autonomous wall-following control
- Pi sends high-level event commands (START, STOP, AVOID_RED)
- Communication frequency: event-driven (low frequency)

This module defines constants and packet handling for both directions:
- Pi → Arduino: Command packets
- Arduino → Pi: Status reports
"""

from typing import Optional, Tuple
import struct

# ============================================================================
# Pi → Arduino 指令 (事件驅動)
# ============================================================================
CMD_START = 0x01        # 開始自主沿牆控制
CMD_STOP = 0x02         # 停止執行
CMD_AVOID_RED = 0x03    # 迴避紅色區域
CMD_SET_VACUUM = 0x04   # 設定吸塵器開關 (payload: 0x00 關閉, 0x01 開啟)
CMD_QUERY_STATE = 0x05  # 查詢當前狀態 (無 payload)

# ============================================================================
# Arduino → Pi 狀態回報
# ============================================================================
STATE_IDLE = 0x00       # 空閒/待命
STATE_FIND_WALL = 0x01  # 搜尋牆面中
STATE_FORWARD = 0x02    # 前進並沿牆
STATE_BACKUP = 0x03     # 後退
STATE_TURN_LEFT = 0x04  # 左轉
STATE_DONE = 0x05       # 任務完成
STATE_ERROR = 0xFF      # 錯誤狀態

# ============================================================================
# 封包格式常數
# ============================================================================

# Pi → Arduino 指令封包
#   [HEADER=0xAA][CMD][PAYLOAD_LEN][PAYLOAD][CHECKSUM][FOOTER=0x55]
#   HEADER      : 1 byte (0xAA)
#   CMD         : 1 byte (指令類型)
#   PAYLOAD_LEN : 1 byte (payload 長度, 0-255)
#   PAYLOAD     : 0-255 bytes (指令參數)
#   CHECKSUM    : 1 byte (CMD ^ PAYLOAD_LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n])
#   FOOTER      : 1 byte (0x55)
#   總長度      : 5 + PAYLOAD_LEN bytes

PKT_HEADER_CMD = 0xAA
PKT_FOOTER_CMD = 0x55

# Arduino → Pi 狀態回報封包
#   [HEADER=0xBB][STATE][CORNER_COUNT][FRONT_DIST_H][FRONT_DIST_L]
#   [RIGHT_DIST_H][RIGHT_DIST_L][YAW_H][YAW_L][FLAGS][CHECKSUM][FOOTER=0x66]
#
#   HEADER       : 1 byte (0xBB)
#   STATE        : 1 byte (當前狀態代碼)
#   CORNER_COUNT : 1 byte (已掃描角落數)
#   FRONT_DIST_H : 1 byte (前方距離高位)
#   FRONT_DIST_L : 1 byte (前方距離低位)
#   RIGHT_DIST_H : 1 byte (右方距離高位)
#   RIGHT_DIST_L : 1 byte (右方距離低位)
#   YAW_H        : 1 byte (偏航角高位)
#   YAW_L        : 1 byte (偏航角低位)
#   FLAGS        : 1 byte (狀態旗標)
#                  bit 0: 偵測到紅色 (1=是)
#                  bit 1: 達到目標 (1=是)
#                  bit 2: 吸塵器啟用 (1=啟用)
#                  bit 3: 傳感器故障 (1=故障)
#   CHECKSUM     : 1 byte (所有資料位的 XOR)
#   FOOTER       : 1 byte (0x66)
#   總長度       : 12 bytes (固定)

PKT_HEADER_STATE = 0xBB
PKT_FOOTER_STATE = 0x66
PKT_STATE_LENGTH = 12   # Arduino → Pi 狀態封包固定長度
PKT_CMD_MIN_LENGTH = 5  # Pi → Arduino 最小長度 (header + cmd + payload_len + checksum + footer)
PKT_CMD_MAX_LENGTH = 260  # Pi → Arduino 最大長度 (header + cmd + len + 255 payload + checksum + footer)

# ============================================================================
# FLAGS 位定義
# ============================================================================
FLAG_RED_DETECTED = 0x01    # bit 0: 偵測到紅色
FLAG_TARGET_REACHED = 0x02  # bit 1: 達到目標
FLAG_VACUUM_ENABLED = 0x04  # bit 2: 吸塵器啟用
FLAG_SENSOR_ERROR = 0x08    # bit 3: 傳感器故障

# ============================================================================
# Payload 常數
# ============================================================================
VACUUM_OFF = 0x00
VACUUM_ON = 0x01

# ============================================================================
# 狀態名稱映射 (用於日誌/調試)
# ============================================================================
STATE_NAMES = {
    STATE_IDLE: "IDLE",
    STATE_FIND_WALL: "FIND_WALL",
    STATE_FORWARD: "FORWARD",
    STATE_BACKUP: "BACKUP",
    STATE_TURN_LEFT: "TURN_LEFT",
    STATE_DONE: "DONE",
    STATE_ERROR: "ERROR",
}

CMD_NAMES = {
    CMD_START: "START",
    CMD_STOP: "STOP",
    CMD_AVOID_RED: "AVOID_RED",
    CMD_SET_VACUUM: "SET_VACUUM",
    CMD_QUERY_STATE: "QUERY_STATE",
}

# ============================================================================
# Checksum 和驗證函數
# ============================================================================

def calc_cmd_checksum(cmd: int, payload: bytes = b'') -> int:
    """
    計算 Pi → Arduino 指令封包的 checksum
    checksum = CMD ^ len(payload) ^ payload[0] ^ ... ^ payload[n]

    Args:
        cmd: 指令代碼 (0-255)
        payload: 指令參數 (bytes, 可為空)

    Returns:
        checksum 值 (0-255)
    """
    checksum = cmd ^ len(payload)
    for byte in payload:
        checksum ^= byte
    return checksum


def calc_state_checksum(state_data: bytes) -> int:
    """
    計算 Arduino → Pi 狀態回報的 checksum
    checksum = XOR of all data bytes (STATE through FLAGS, excluding HEADER and FOOTER)

    Args:
        state_data: 狀態回報的資料部分 (不含 header 和 footer, 10 bytes)
                   [STATE][CORNER_COUNT][FRONT_H][FRONT_L][RIGHT_H][RIGHT_L][YAW_H][YAW_L][FLAGS]

    Returns:
        checksum 值 (0-255)
    """
    checksum = 0
    for byte in state_data:
        checksum ^= byte
    return checksum


# ============================================================================
# 封包構建和解析
# ============================================================================

class CommandPacket:
    """Pi → Arduino 指令封包"""

    def __init__(self, cmd: int, payload: bytes = b''):
        """
        初始化指令封包

        Args:
            cmd: 指令代碼 (CMD_START, CMD_STOP, etc.)
            payload: 指令參數 (bytes)
        """
        if cmd < 0 or cmd > 255:
            raise ValueError(f"cmd 必須在 0-255 之間，得到 {cmd}")
        if len(payload) > 255:
            raise ValueError(f"payload 不能超過 255 bytes，得到 {len(payload)}")

        self.cmd = cmd
        self.payload = payload

    def serialize(self) -> bytes:
        """
        序列化為二進位格式

        Returns:
            完整的封包 (bytes)
        """
        checksum = calc_cmd_checksum(self.cmd, self.payload)
        packet = bytes([
            PKT_HEADER_CMD,
            self.cmd,
            len(self.payload),
        ]) + self.payload + bytes([
            checksum,
            PKT_FOOTER_CMD,
        ])
        return packet

    def __repr__(self) -> str:
        cmd_name = CMD_NAMES.get(self.cmd, f"UNKNOWN(0x{self.cmd:02X})")
        return f"CommandPacket(cmd={cmd_name}, payload_len={len(self.payload)})"


class StatePacket:
    """Arduino → Pi 狀態回報封包"""

    def __init__(self, state: int, corner_count: int,
                 front_distance: int, right_distance: int, yaw: int, flags: int):
        """
        初始化狀態回報封包

        Args:
            state: 當前狀態代碼 (STATE_IDLE, STATE_FORWARD, etc.)
            corner_count: 已掃描角落數 (0-255)
            front_distance: 前方距離 (0-65535, cm)
            right_distance: 右方距離 (0-65535, cm)
            yaw: 偏航角 (0-65535, 0.1° 單位)
            flags: 狀態旗標 (0-255)
        """
        self.state = state
        self.corner_count = corner_count
        self.front_distance = front_distance
        self.right_distance = right_distance
        self.yaw = yaw
        self.flags = flags

    def serialize(self) -> bytes:
        """
        序列化為二進位格式

        Returns:
            完整的封包 (bytes, 長度為 12)
        """
        data = bytes([
            self.state,
            self.corner_count,
            (self.front_distance >> 8) & 0xFF,
            self.front_distance & 0xFF,
            (self.right_distance >> 8) & 0xFF,
            self.right_distance & 0xFF,
            (self.yaw >> 8) & 0xFF,
            self.yaw & 0xFF,
            self.flags,
        ])
        checksum = calc_state_checksum(data)
        packet = bytes([PKT_HEADER_STATE]) + data + bytes([checksum, PKT_FOOTER_STATE])
        return packet

    @staticmethod
    def deserialize(packet: bytes) -> Optional['StatePacket']:
        """
        從二進位格式反序列化

        Args:
            packet: 完整的狀態封包 (bytes, 長度必須為 12)

        Returns:
            StatePacket 物件，如果驗證失敗則返回 None
        """
        if not verify_state_packet(packet):
            return None

        state = packet[1]
        corner_count = packet[2]
        front_distance = (packet[3] << 8) | packet[4]
        right_distance = (packet[5] << 8) | packet[6]
        yaw = (packet[7] << 8) | packet[8]
        flags = packet[9]

        return StatePacket(state, corner_count, front_distance, right_distance, yaw, flags)

    def has_flag(self, flag: int) -> bool:
        """檢查是否設置特定旗標"""
        return (self.flags & flag) != 0

    def get_state_name(self) -> str:
        """取得狀態名稱"""
        return STATE_NAMES.get(self.state, f"UNKNOWN(0x{self.state:02X})")

    def __repr__(self) -> str:
        state_name = self.get_state_name()
        return (f"StatePacket(state={state_name}, corners={self.corner_count}, "
                f"front={self.front_distance}cm, right={self.right_distance}cm, "
                f"yaw={self.yaw*0.1:.1f}°, flags=0x{self.flags:02X})")


# ============================================================================
# 封包驗證
# ============================================================================

def verify_cmd_packet(packet: bytes) -> bool:
    """
    驗證 Pi → Arduino 指令封包
    檢查 header, footer, checksum

    Args:
        packet: 封包 (bytes)

    Returns:
        True 如果有效，False 否則
    """
    if len(packet) < PKT_CMD_MIN_LENGTH:
        return False

    if packet[0] != PKT_HEADER_CMD:
        return False

    if packet[-1] != PKT_FOOTER_CMD:
        return False

    cmd = packet[1]
    payload_len = packet[2]

    # 驗證長度
    expected_length = 5 + payload_len
    if len(packet) != expected_length:
        return False

    # 驗證 checksum
    payload = packet[3:3+payload_len]
    expected_checksum = calc_cmd_checksum(cmd, payload)
    actual_checksum = packet[3 + payload_len]

    return expected_checksum == actual_checksum


def verify_state_packet(packet: bytes) -> bool:
    """
    驗證 Arduino → Pi 狀態回報封包
    檢查 header, footer, checksum, 固定長度

    Args:
        packet: 封包 (bytes)

    Returns:
        True 如果有效，False 否則
    """
    if len(packet) != PKT_STATE_LENGTH:
        return False

    if packet[0] != PKT_HEADER_STATE:
        return False

    if packet[-1] != PKT_FOOTER_STATE:
        return False

    # 驗證 checksum
    data = packet[1:10]  # STATE through FLAGS
    expected_checksum = calc_state_checksum(data)
    actual_checksum = packet[10]

    return expected_checksum == actual_checksum


# ============================================================================
# 便利函數：常用指令建立
# ============================================================================

def create_cmd_start() -> CommandPacket:
    """建立 START 指令"""
    return CommandPacket(CMD_START)


def create_cmd_stop() -> CommandPacket:
    """建立 STOP 指令"""
    return CommandPacket(CMD_STOP)


def create_cmd_avoid_red() -> CommandPacket:
    """建立 AVOID_RED 指令"""
    return CommandPacket(CMD_AVOID_RED)


def create_cmd_set_vacuum(enabled: bool) -> CommandPacket:
    """
    建立 SET_VACUUM 指令

    Args:
        enabled: True = 開啟, False = 關閉
    """
    payload = bytes([VACUUM_ON if enabled else VACUUM_OFF])
    return CommandPacket(CMD_SET_VACUUM, payload)


def create_cmd_query_state() -> CommandPacket:
    """建立 QUERY_STATE 指令"""
    return CommandPacket(CMD_QUERY_STATE)


if __name__ == "__main__":
    # 示例和測試
    print("=== Pi → Arduino 指令範例 ===")

    # START 指令
    cmd_start = create_cmd_start()
    print(f"{cmd_start}")
    packet = cmd_start.serialize()
    print(f"  序列化: {' '.join(f'0x{b:02X}' for b in packet)}")
    print(f"  驗證: {verify_cmd_packet(packet)}")

    # SET_VACUUM 指令
    cmd_vacuum = create_cmd_set_vacuum(True)
    print(f"\n{cmd_vacuum}")
    packet = cmd_vacuum.serialize()
    print(f"  序列化: {' '.join(f'0x{b:02X}' for b in packet)}")
    print(f"  驗證: {verify_cmd_packet(packet)}")

    print("\n=== Arduino → Pi 狀態回報範例 ===")

    # 狀態回報
    state_packet = StatePacket(
        state=STATE_FORWARD,
        corner_count=2,
        front_distance=150,
        right_distance=45,
        yaw=1234,  # 123.4°
        flags=FLAG_VACUUM_ENABLED
    )
    print(f"{state_packet}")
    packet = state_packet.serialize()
    print(f"  序列化: {' '.join(f'0x{b:02X}' for b in packet)}")
    print(f"  驗證: {verify_state_packet(packet)}")

    # 反序列化
    restored = StatePacket.deserialize(packet)
    print(f"  反序列化: {restored}")
    print(f"  狀態: {restored.get_state_name()}")
    print(f"  吸塵器啟用: {restored.has_flag(FLAG_VACUUM_ENABLED)}")
