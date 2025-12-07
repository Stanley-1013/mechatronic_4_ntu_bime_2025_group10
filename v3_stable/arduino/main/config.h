#ifndef CONFIG_H
#define CONFIG_H

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
#define LOOP_INTERVAL   50      // 50ms = 20Hz

// 超聲波
#define US_TIMEOUT      8000    // 8ms 超時 (~136cm)
#define US_MIN_VALID    2       // 最小有效值 cm
#define US_MAX_VALID    200     // 最大有效值 cm

// 沿牆行為 (距離 + 角度 PD 控制)
#define TARGET_DIST     15.0f   // 目標右側距離 cm
#define TARGET_ANGLE    0.0f    // 目標角度 (0 = 平行牆壁)
#define KP_DIST         0.03f   // 距離比例增益
#define KP_ANGLE        0.02f   // 角度比例增益 (度 → angular)
#define KD_ANGLE        0.012f  // 角度微分增益 (預測趨勢) v3.2: 0.015→0.012 減少晃動
#define MAX_ANGULAR     0.4f    // 最大轉向幅度
#define SEARCH_ANGULAR  0.05f   // 找牆時的右弧線幅度 (非常輕微)

// 前方避障
#define FRONT_STOP      15.0f   // 觸發轉彎距離
#define FRONT_SLOW      30.0f   // 減速距離

// 轉彎行為 (基於前方暢通 + 右側平行)
#define TURN_FRONT_CLEAR    40.0f   // 轉彎完成：前方暢通
#define TURN_RIGHT_MIN      10.0f   // 轉彎完成：右側最小
#define TURN_RIGHT_MAX      40.0f   // 轉彎完成：右側最大
#define TURN_ANGLE_TOL      10.0f   // 轉彎完成：角度容許誤差 (度)
#define TURN_MIN_TIME       6       // 轉彎最少時間 (300ms)
#define TURN_TIMEOUT        50      // 轉彎超時 (2.5秒)
#define TURN_STABLE         12      // 穩定期 (600ms)

// 馬達
#define BASE_PWM        60      // 基礎 PWM
#define TURN_PWM        55      // 原地轉彎 PWM
#define MIN_PWM         45      // 最小有效 PWM
#define LEFT_SCALE      1.0f    // 左輪比例 (調校用)
#define RIGHT_SCALE     1.0f    // 右輪比例 (調校用)

// 超時保護
#define RUN_TIMEOUT     300000  // 5 分鐘

#endif
