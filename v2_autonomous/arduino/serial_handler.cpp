#include "serial_handler.h"

SerialHandler::SerialHandler()
    : _callback(nullptr), _rxIndex(0), _rxState(RX_WAIT_HEADER),
      _currentCmd(0), _payloadLen(0), _payloadIndex(0) {
}

void SerialHandler::begin(unsigned long baudRate) {
    Serial.begin(baudRate);
    _resetRx();
}

void SerialHandler::setCommandCallback(CommandCallback callback) {
    _callback = callback;
}

void SerialHandler::_resetRx() {
    _rxState = RX_WAIT_HEADER;
    _rxIndex = 0;
    _currentCmd = 0;
    _payloadLen = 0;
    _payloadIndex = 0;
}

void SerialHandler::process() {
    // 非阻塞式接收：每個 loop 只處理可用的 bytes
    while (Serial.available() > 0) {
        uint8_t byte = Serial.read();

        switch (_rxState) {
            case RX_WAIT_HEADER:
                if (byte == PKT_HEADER_CMD) {
                    // 防止緩衝區溢出：在寫入前檢查
                    if (_rxIndex >= sizeof(_rxBuffer)) {
                        _resetRx();
                        break;
                    }
                    _rxBuffer[_rxIndex] = byte;
                    _rxIndex++;
                    _rxState = RX_READ_CMD;
                }
                break;

            case RX_READ_CMD:
                // 防止緩衝區溢出：在寫入前檢查
                if (_rxIndex >= sizeof(_rxBuffer)) {
                    _resetRx();
                    break;
                }
                _rxBuffer[_rxIndex] = byte;
                _rxIndex++;
                _currentCmd = byte;
                _rxState = RX_READ_LEN;
                break;

            case RX_READ_LEN:
                // 防止緩衝區溢出：在寫入前檢查
                if (_rxIndex >= sizeof(_rxBuffer)) {
                    _resetRx();
                    break;
                }
                _rxBuffer[_rxIndex] = byte;
                _rxIndex++;
                _payloadLen = byte;

                // 檢查 payload 長度是否合理
                if (_payloadLen > 255) {
                    _resetRx();
                    break;
                }

                // 檢查緩衝區容量 (header + cmd + len + payload + checksum + footer)
                // 總共需要: 1 + 1 + 1 + payload_len + 1 + 1 = 5 + payload_len
                if (_payloadLen > (sizeof(_rxBuffer) - 5)) {
                    _resetRx();
                    break;
                }

                if (_payloadLen == 0) {
                    // 無 payload，直接跳到 checksum
                    _rxState = RX_READ_CHECKSUM;
                } else {
                    // 有 payload，準備接收
                    _payloadIndex = 0;
                    _rxState = RX_READ_PAYLOAD;
                }
                break;

            case RX_READ_PAYLOAD:
                // 防止緩衝區溢出：在寫入前檢查
                if (_rxIndex >= sizeof(_rxBuffer)) {
                    _resetRx();
                    break;
                }
                _rxBuffer[_rxIndex] = byte;
                _rxIndex++;
                _payloadIndex++;

                if (_payloadIndex >= _payloadLen) {
                    // payload 接收完成，進入 checksum 讀取
                    _rxState = RX_READ_CHECKSUM;
                }
                break;

            case RX_READ_CHECKSUM:
                // 防止緩衝區溢出：在寫入前檢查
                if (_rxIndex >= sizeof(_rxBuffer)) {
                    _resetRx();
                    break;
                }
                _rxBuffer[_rxIndex] = byte;
                _rxIndex++;

                // 驗證 checksum: CMD ^ PAYLOAD_LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n]
                uint8_t expectedChecksum = calc_cmd_checksum(_currentCmd, _payloadLen,
                                                               _payloadLen > 0 ? &_rxBuffer[3] : nullptr);
                if (byte != expectedChecksum) {
                    // checksum 失敗，重置
                    _resetRx();
                    break;
                }

                _rxState = RX_READ_FOOTER;
                break;

            case RX_READ_FOOTER:
                // 防止緩衝區溢出：在寫入前檢查
                if (_rxIndex >= sizeof(_rxBuffer)) {
                    _resetRx();
                    break;
                }
                _rxBuffer[_rxIndex] = byte;
                _rxIndex++;

                if (byte == PKT_FOOTER_CMD) {
                    // 完整的封包接收成功
                    _processPacket();
                } else {
                    // footer 不匹配
                    // 可能是同步錯誤，重置狀態機
                }

                _resetRx();
                break;
        }
    }
}

void SerialHandler::_processPacket() {
    // 此時 _rxBuffer 包含完整的封包
    // 格式: [HEADER][CMD][PAYLOAD_LEN][PAYLOAD...][CHECKSUM][FOOTER]

    // 使用 protocol 的驗證函數再次驗證（雙重檢查）
    if (!verify_cmd_packet(_rxBuffer, _rxIndex)) {
        return;
    }

    // 提取 cmd 和 payload
    uint8_t cmd = _rxBuffer[1];
    uint8_t payloadLen = _rxBuffer[2];
    uint8_t* payload = payloadLen > 0 ? &_rxBuffer[3] : nullptr;

    // 呼叫回調函數
    if (_callback != nullptr) {
        _callback(cmd, payloadLen, payload);
    }
}

void SerialHandler::sendState(uint8_t state, uint8_t cornerCount,
                               uint16_t frontDist, uint16_t rightDist,
                               int16_t yaw, uint8_t flags) {
    // 構建狀態回報封包
    // 格式: [HEADER=0xBB][STATE][CORNER_COUNT][FRONT_H][FRONT_L][RIGHT_H][RIGHT_L]
    //       [YAW_H][YAW_L][FLAGS][CHECKSUM][FOOTER=0x66]

    uint8_t packet[PKT_STATE_LENGTH];
    uint8_t index = 0;

    // Header
    packet[index++] = PKT_HEADER_STATE;  // 0xBB

    // 狀態資訊 (9 個位元組)
    packet[index++] = state;                     // STATE
    packet[index++] = cornerCount;               // CORNER_COUNT
    packet[index++] = (frontDist >> 8) & 0xFF;  // FRONT_DIST_H
    packet[index++] = frontDist & 0xFF;         // FRONT_DIST_L
    packet[index++] = (rightDist >> 8) & 0xFF;  // RIGHT_DIST_H
    packet[index++] = rightDist & 0xFF;         // RIGHT_DIST_L
    packet[index++] = (yaw >> 8) & 0xFF;        // YAW_H (signed 處理)
    packet[index++] = yaw & 0xFF;               // YAW_L
    packet[index++] = flags;                     // FLAGS

    // Checksum (計算 packet[1] 到 packet[9] 的 XOR)
    uint8_t checksum = calc_state_checksum(&packet[1]);
    packet[index++] = checksum;

    // Footer
    packet[index++] = PKT_FOOTER_STATE;  // 0x66

    // 發送封包 (不使用 flush()，避免阻塞)
    Serial.write(packet, PKT_STATE_LENGTH);
}
