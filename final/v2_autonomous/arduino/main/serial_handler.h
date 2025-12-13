#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <Arduino.h>
#include "protocol.h"

// 回調函數型別
typedef void (*CommandCallback)(uint8_t cmd, uint8_t payloadLen, uint8_t* payload);

class SerialHandler {
public:
    SerialHandler();

    void begin(unsigned long baudRate);
    void setCommandCallback(CommandCallback callback);

    // 主處理函數（每個 loop 呼叫）
    void process();

    // 發送狀態封包
    void sendState(uint8_t state, uint8_t cornerCount,
                   uint16_t frontDist, uint16_t rightDist,
                   int16_t yaw, uint8_t flags);

private:
    CommandCallback _callback;

    // 接收緩衝區
    uint8_t _rxBuffer[32];
    uint8_t _rxIndex;

    // 解析狀態機
    enum RxState { RX_WAIT_HEADER, RX_READ_CMD, RX_READ_LEN, RX_READ_PAYLOAD, RX_READ_CHECKSUM, RX_READ_FOOTER };
    RxState _rxState;
    uint8_t _currentCmd;
    uint8_t _payloadLen;
    uint8_t _payloadIndex;

    void _resetRx();
    void _processPacket();
};

#endif
