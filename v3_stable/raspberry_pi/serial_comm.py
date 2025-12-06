# serial_comm.py - Serial 通訊模組

import serial
from config import *

class SerialComm:
    def __init__(self):
        self.ser = None

    def connect(self):
        """連接 Arduino"""
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            return True
        except Exception as e:
            print(f"Serial 連接失敗: {e}")
            return False

    def send_command(self, cmd):
        """發送指令封包"""
        if self.ser is None:
            return False

        checksum = PKT_HEADER ^ cmd
        packet = bytes([PKT_HEADER, cmd, checksum, PKT_FOOTER])

        try:
            self.ser.write(packet)
            return True
        except Exception as e:
            print(f"發送失敗: {e}")
            return False

    def vacuum_on(self):
        """開啟吸塵器"""
        return self.send_command(CMD_VACUUM_ON)

    def vacuum_off(self):
        """關閉吸塵器"""
        return self.send_command(CMD_VACUUM_OFF)

    def close(self):
        """關閉連接"""
        if self.ser:
            self.ser.close()
