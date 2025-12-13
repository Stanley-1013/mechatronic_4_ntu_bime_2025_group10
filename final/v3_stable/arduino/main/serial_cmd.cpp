#include "serial_cmd.h"
#include <Arduino.h>

void SerialCommand::init(long baudRate) {
    Serial.begin(baudRate);
    _bufIndex = 0;
    _lastCommand = 0;
    _hasCommand = false;
}

bool SerialCommand::check() {
    _hasCommand = false;

    while (Serial.available()) {
        byte b = Serial.read();

        // 尋找封包頭
        if (_bufIndex == 0) {
            if (b == PKT_HEADER) {
                _buffer[_bufIndex++] = b;
            }
            continue;
        }

        _buffer[_bufIndex++] = b;

        // 封包完成 (4 bytes: Header, Cmd, Checksum, Footer)
        if (_bufIndex >= 4) {
            // 驗證 Footer
            if (_buffer[3] == PKT_FOOTER) {
                // 驗證 Checksum
                byte checksum = _buffer[0] ^ _buffer[1];
                if (checksum == _buffer[2]) {
                    _lastCommand = _buffer[1];
                    _hasCommand = true;
                }
            }
            _bufIndex = 0;
        }
    }

    return _hasCommand;
}

byte SerialCommand::getCommand() {
    return _lastCommand;
}
