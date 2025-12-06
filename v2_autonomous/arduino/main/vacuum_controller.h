/*
 * vacuum_controller.h - 吸塵器馬達控制模組
 * 版本: 1.0
 * 日期: 2025-10-31
 *
 * 控制吸塵器馬達開關。
 * 參考: 01_SRS_軟體需求規格書.md - FR5
 */

#ifndef VACUUM_CONTROLLER_H
#define VACUUM_CONTROLLER_H

#include <Arduino.h>

class VacuumController {
public:
    /**
     * @brief 建構子
     * @param control_pin 控制腳位（D12）
     */
    VacuumController(uint8_t control_pin);

    /**
     * @brief 初始化吸塵器控制（設定腳位模式）
     */
    void begin();

    /**
     * @brief 設定吸塵器狀態
     * @param state true = 開啟，false = 關閉
     */
    void setState(bool state);

    /**
     * @brief 取得吸塵器狀態
     * @return true = 開啟，false = 關閉
     */
    bool getState() const;

private:
    uint8_t _control_pin;
    bool _state;
};

#endif // VACUUM_CONTROLLER_H
