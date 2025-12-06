// behavior.cpp - 優先級覆蓋控制
#include "behavior.h"
#include "config.h"
#include <Arduino.h>

void BehaviorController::init() {
    _cornerCount = 0;
    _isTurning = false;
    _turnConfirm = 0;
    _turnTimer = 0;
    _stableTimer = 0;
    _frontConfirm = 0;
    _exitCounter = 0;
    _complete = false;
    _startTime = millis();
    _exitSearchStart = 0;  // 出場搜尋尚未開始
    _lastRightDist = TARGET_DIST;  // 初始化為目標距離

    // 啟用 Watchdog (2 秒)
    wdt_enable(WDTO_2S);
}

MotorCommand BehaviorController::update(int frontDist, int rightDist) {
    // 餵狗
    wdt_reset();

    // 超時保護
    if (millis() - _startTime > RUN_TIMEOUT) {
        _complete = true;
        return {0, 0, true};
    }

    // 已完成
    if (_complete) {
        return {0, 0, true};
    }

    // ===== 穩定期：剛轉完彎，維持直走（不受右側影響）=====
    if (_stableTimer > 0) {
        _stableTimer--;
        return _handleWallFollow(frontDist, rightDist, true);  // stabilizing=true
    }

    // ===== 優先級 1: 角落轉彎（需確認防誤觸發） =====
    if (frontDist < FRONT_STOP) {
        _frontConfirm++;
    } else {
        _frontConfirm = 0;
    }

    if (_isTurning || _frontConfirm >= FRONT_CONFIRM) {
        return _handleTurning(frontDist, rightDist);
    }

    // ===== 優先級 2: 出場 =====
    _updateExitCounter(frontDist, rightDist);
    if (_exitCounter >= EXIT_CONFIRM) {
        return _handleExit(frontDist, rightDist);
    }

    // ===== 優先級 3: 沿牆 + 減速避障 =====
    return _handleWallFollow(frontDist, rightDist);
}

// ===== 優先級 1: 原地轉彎 =====
MotorCommand BehaviorController::_handleTurning(int frontDist, int rightDist) {
    // 首次進入轉彎
    if (!_isTurning) {
        _isTurning = true;
        _turnTimer = 0;
        _turnConfirm = 0;
    }

    // 轉彎計時 (超時保護)
    _turnTimer++;
    if (_turnTimer > TURN_TIMEOUT) {
        // 超時強制結束轉彎，視為完成一角
        _isTurning = false;
        _turnTimer = 0;
        _turnConfirm = 0;
        _stableTimer = TURN_STABLE;
        _cornerCount++;
        // 第 4 角完成後開始出場搜尋窗口
        if (_cornerCount == 4) {
            _exitSearchStart = millis();
        }
        return _handleWallFollow(frontDist, rightDist);
    }

    // 最少轉彎時間：防止過早結束導致連續卡角
    if (_turnTimer < TURN_MIN_TIME) {
        return {-TURN_PWM, +TURN_PWM, false};
    }

    // 轉彎完成條件（必須同時滿足）：
    // 1. 前方暢通 (>45cm) - 確保已離開角落
    // 2. 右側在合理沿牆範圍 (10~30cm) - 確保轉到正確角度
    bool frontCleared = (frontDist > TURN_COMPLETE_FRONT);
    bool rightInRange = (rightDist > WALL_DANGER + 2 && rightDist < TURN_DETECT_DIST);

    if (frontCleared && rightInRange) {
        // 最佳條件：快速確認
        _turnConfirm += 2;
    } else if (frontCleared && rightDist > TURN_DETECT_DIST) {
        // 前方暢通但右側太遠（開放空間）：慢慢確認
        _turnConfirm++;
    } else {
        _turnConfirm = 0;
    }

    // 需累積 4 點確認
    if (_turnConfirm >= 4) {
        _isTurning = false;
        _turnTimer = 0;
        _turnConfirm = 0;
        _stableTimer = TURN_STABLE;
        _cornerCount++;
        // 第 4 角完成後開始出場搜尋窗口
        if (_cornerCount == 4) {
            _exitSearchStart = millis();
        }
        return _handleWallFollow(frontDist, rightDist);
    }

    // 原地左轉
    return {-TURN_PWM, +TURN_PWM, false};
}

// ===== 優先級 2: 出場 =====
void BehaviorController::_updateExitCounter(int frontDist, int rightDist) {
    // 時間窗口檢查：第 4 角後 2~15 秒內才搜尋出口
    unsigned long now = millis();
    bool inExitWindow = (_exitSearchStart > 0) &&
                        (now - _exitSearchStart >= EXIT_WINDOW_MIN) &&
                        (now - _exitSearchStart <= EXIT_WINDOW_MAX);

    // 多條件確認
    bool exitCondition =
        _cornerCount >= 4 &&
        inExitWindow &&                       // 在有效時間窗口內
        rightDist > EXIT_THRESHOLD &&         // 右側無牆 (>50cm)
        frontDist > EXIT_FRONT_CLEAR &&       // 前方暢通 (>60cm)
        _stableTimer == 0 &&                  // 不在穩定期
        !_isTurning;

    if (exitCondition) {
        _exitCounter += 1.0f;
    } else if (_exitCounter > 0 && _exitCounter < EXIT_CONFIRM) {
        // 容許短暫中斷，緩慢衰減
        _exitCounter -= 0.5f;
        if (_exitCounter < 0) _exitCounter = 0;
    } else {
        _exitCounter = 0;
    }
}

MotorCommand BehaviorController::_handleExit(int frontDist, int rightDist) {
    // 階段 1: 右轉朝向出口
    if (_exitCounter < EXIT_TURN) {
        return {+TURN_PWM, -TURN_PWM/2, false};
    }

    // 階段 2: 直走出場
    if (_exitCounter < EXIT_TOTAL) {
        _exitCounter += 1.0f;
        return {BASE_PWM, BASE_PWM, false};
    }

    // 階段 3: 停止
    _complete = true;
    return {0, 0, true};
}

// ===== 優先級 3: 沿牆 + 找牆 =====
MotorCommand BehaviorController::_handleWallFollow(int frontDist, int rightDist, bool stabilizing) {
    float angular = 0;

    // 穩定期：剛轉完彎，直走為主，只做輕微修正
    if (stabilizing) {
        // 只有太遠才輕微右轉靠回，太近不推離（避免多轉）
        if (rightDist > TARGET_DIST + 10) {
            angular = -0.05f;  // 輕微右轉
        }
        // 其他情況直走
    }
    else {
        // 正常沿牆：PD 控制
        float error = TARGET_DIST - rightDist;
        float derivative = rightDist - _lastRightDist;  // 正=遠離中, 負=靠近中

        // P 項
        float p_term = 0;
        if (rightDist > WALL_MAX_DIST) {
            // 無牆區：右弧線找牆
            float searchStrength = SEARCH_ANGULAR;
            if (frontDist < FRONT_SLOW) {
                searchStrength *= (frontDist - FRONT_STOP) / (FRONT_SLOW - FRONT_STOP);
                if (searchStrength < 0) searchStrength = 0;
            }
            p_term = -searchStrength;
        }
        else if (rightDist > TARGET_DIST) {
            // 太遠區：拉回 + 找牆
            float ratio = (rightDist - TARGET_DIST) / (WALL_MAX_DIST - TARGET_DIST);
            float pullBack = KP_WALL * 0.7f * error;
            float search = -SEARCH_ANGULAR * ratio;
            p_term = pullBack * (1.0f - ratio) + search;
        }
        else if (rightDist > WALL_DANGER) {
            // 正常區
            float ratio = (TARGET_DIST - rightDist) / (TARGET_DIST - WALL_DANGER);
            float kp = KP_WALL + ratio * (KP_WALL_DANGER - KP_WALL);
            p_term = kp * error;
        }
        else {
            // 危險區
            p_term = KP_WALL_DANGER * error;
        }

        // D 項：正在遠離時減弱推力，正在靠近時加強
        float d_term = -KD_WALL * derivative;

        angular = p_term + d_term;

        // 限幅
        if (angular > MAX_ANGULAR) angular = MAX_ANGULAR;
        if (angular < -MAX_ANGULAR) angular = -MAX_ANGULAR;

        // 更新上次距離
        _lastRightDist = rightDist;
    }

    // 前方減速（接近角落時減速，但不轉向）
    float speedScale = 1.0f;
    if (frontDist < FRONT_SLOW) {
        speedScale = 0.6f + 0.4f * (frontDist / FRONT_SLOW);
    }

    // 轉換為 PWM
    int basePWM = (int)(BASE_PWM * speedScale);
    int leftPWM = (int)(basePWM * (1.0f - angular));
    int rightPWM = (int)(basePWM * (1.0f + angular));

    // 應用馬達比例校正
    leftPWM = (int)(leftPWM * LEFT_SCALE);
    rightPWM = (int)(rightPWM * RIGHT_SCALE);

    // 限幅
    if (leftPWM > 0 && leftPWM < MIN_PWM) leftPWM = MIN_PWM;
    if (rightPWM > 0 && rightPWM < MIN_PWM) rightPWM = MIN_PWM;
    if (leftPWM > 255) leftPWM = 255;
    if (rightPWM > 255) rightPWM = 255;

    return {leftPWM, rightPWM, false};
}
