#ifndef CONFIG_H
#define CONFIG_H

// ============ DEBUG 巨集 ============
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)

// ============ 腳位定義 ============
// L298N 馬達驅動
#define PIN_ENA     3   // 左輪 PWM
#define PIN_IN1     6   // 左輪方向 A
#define PIN_IN2     5   // 左輪方向 B
#define PIN_ENB     11  // 右輪 PWM
#define PIN_IN3     10  // 右輪方向 A
#define PIN_IN4     9   // 右輪方向 B

// 超聲波 (3 個)
#define PIN_FRONT_TRIG      7
#define PIN_FRONT_ECHO      8
#define PIN_RIGHT_F_TRIG    A1      // 右前
#define PIN_RIGHT_F_ECHO    A2
#define PIN_RIGHT_R_TRIG    2       // 右後 (新增)
#define PIN_RIGHT_R_ECHO    4

// 吸塵器繼電器
#define PIN_VACUUM      A3

// ============ 雙超音波參數 ============
#define US_SPACING      15.0f   // 右側兩超音波間距 (cm)，需實際測量

// ============ 控制參數 ============
// 主迴圈
#define LOOP_INTERVAL   30      // 30ms = 33Hz (v3.12: 加快響應)

// 超聲波
#define US_TIMEOUT      8000    // 8ms 超時 (~136cm)
#define US_MIN_VALID    2       // 最小有效值 cm
#define US_MAX_VALID    200     // 最大有效值 cm

// 沿牆行為 (v3.15: 純角度 PD，距離影響目標角度)
#define TARGET_DIST     15.0f   // 目標右側距離 cm
#define KP_DIST         0.5f    // 距離→目標角度 (7cm差=3.5°目標角度)
#define KP_ANGLE        0.04f   // 角度比例增益 (v3.15: 0.035→0.04)
#define KD_ANGLE        0.02f   // 角度微分增益
#define MAX_TARGET_ANGLE 10.0f  // 目標角度限幅 (度)
#define MAX_ANGULAR     0.6f    // 最大轉向幅度
#define SEARCH_ANGULAR  0.05f   // 找牆時的右弧線幅度 (非常輕微)

// 前方避障
#define FRONT_STOP      16.0f   // 觸發轉彎距離
#define FRONT_SLOW      30.0f   // 減速距離

// 轉彎行為 (基於前方暢通 + 右側平行)
#define TURN_FRONT_CLEAR    30.0f   // 轉彎完成：前方暢通 (原40，降低提早退出)
#define TURN_RIGHT_MIN      10.0f   // 轉彎完成：右側最小
#define TURN_RIGHT_MAX      40.0f   // 轉彎完成：右側最大
#define TURN_ANGLE_TOL      10.0f   // 轉彎完成：角度容許誤差 (度)
#define TURN_MIN_TIME       10      // 最少轉彎時間 (10 * 30ms = 300ms)
#define TURN_TIME           24      // 左轉固定時間 (24 * 50ms = 1200ms)
#define TURN_TIMEOUT        50      // 轉彎超時 (1.5秒)
#define TURN_STABLE         12      // 穩定期 (600ms)

// 擺頭清掃 (v3.14: 角落轉彎前原地擺動)
#define SWEEP_ANGLE         30.0f   // 擺頭角度 (度)
#define SWEEP_PWM           70      // 擺頭 PWM (與轉彎相同)
#define SWEEP_TIMEOUT       166     // 擺頭超時 (166 * 30ms ≈ 5秒)

// 馬達（左右獨立，方便調參）
#define BASE_PWM_L      64      // 左輪直走 PWM
#define BASE_PWM_R      77      // 右輪直走 PWM（目前偏右，待調整）
#define TURN_PWM        70      // 轉彎 PWM
#define MIN_PWM         45      // 最小有效 PWM

// 超時保護
#define RUN_TIMEOUT     300000  // 5 分鐘

#endif
