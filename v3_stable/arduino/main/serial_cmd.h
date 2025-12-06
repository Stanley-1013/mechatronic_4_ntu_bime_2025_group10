// serial_cmd.h
#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H

#include <Arduino.h>  // For byte type

// 指令定義
#define CMD_VACUUM_ON   0x01
#define CMD_VACUUM_OFF  0x02

// 封包格式
#define PKT_HEADER      0xAA
#define PKT_FOOTER      0x55

class SerialCommand {
public:
    void init(long baudRate);
    bool check();           // 檢查是否有新指令
    byte getCommand();      // 取得指令

private:
    byte _buffer[4];
    int _bufIndex;
    byte _lastCommand;
    bool _hasCommand;
};

#endif
