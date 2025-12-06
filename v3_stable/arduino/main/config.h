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

// 超聲波
#define PIN_FRONT_TRIG  7
#define PIN_FRONT_ECHO  8
#define PIN_RIGHT_TRIG  A1
#define PIN_RIGHT_ECHO  A2

// 吸塵器繼電器
#define PIN_VACUUM      A3

// ============ 控制參數 ============
// 主迴圈
#define LOOP_INTERVAL   50      // 50ms = 20Hz

// 超聲波
#define US_TIMEOUT      8000    // 8ms 超時 (~136cm)
#define US_MIN_VALID    2       // 最小有效值 cm
#define US_MAX_VALID    200     // 最大有效值 cm

// 沿牆行為
#define TARGET_DIST     15.0f   // 目標右側距離 cm
#define WALL_MIN_DIST   5.0f    // 最近允許距離 cm (太近會撞)
#define WALL_DANGER     10.0f   // 危險距離：強力推離 (放寬至10cm)
#define WALL_MAX_DIST   35.0f   // 超過此距離視為無牆
#define KP_WALL         0.04f   // 基礎比例增益 (加倍)
#define KP_WALL_DANGER  0.12f   // 危險距離增益（大幅加強）
#define KD_WALL         0.04f   // 微分增益
#define MAX_ANGULAR     0.45f   // 最大轉向幅度 (放寬)
#define SEARCH_ANGULAR  0.20f   // 找牆時的右弧線幅度（加強）

// 前方避障（角落轉彎）
#define FRONT_STOP          15.0f   // 停止轉彎距離（貼近角落）
#define FRONT_SLOW          30.0f   // 減速距離
#define FRONT_CONFIRM       2       // 進入轉彎需連續確認次數

// 轉彎行為
#define TURN_DETECT_DIST    40.0f   // 轉彎完成：右側新牆偵測 (放寬)
#define TURN_COMPLETE_FRONT 35.0f   // 轉彎完成：前方暢通 (降低)
#define TURN_MIN_TIME       6       // 轉彎最少時間 (300ms)
#define TURN_TIMEOUT        40      // 轉彎超時 (2秒)
#define TURN_STABLE         8       // 穩定期 (400ms) - 轉彎後直走，不受右側影響

// 出場行為
#define EXIT_THRESHOLD      50.0f   // 右側無牆距離 cm
#define EXIT_FRONT_CLEAR    60.0f   // 前方暢通距離 cm
#define EXIT_WINDOW_MIN     2000    // 出場窗口最小 ms
#define EXIT_WINDOW_MAX     15000   // 出場窗口最大 ms
#define EXIT_CONFIRM        6       // 確認計數 (300ms)
#define EXIT_TURN           22      // 右轉結束 (6+16)
#define EXIT_TOTAL          62      // 總出場 (6+16+40)

// 馬達
#define BASE_PWM        60      // 基礎 PWM (從80降低，增加反應時間)
#define TURN_PWM        60      // 原地轉彎 PWM
#define MIN_PWM         45      // 最小有效 PWM
#define LEFT_SCALE      1.0f    // 左輪比例 (調校用)
#define RIGHT_SCALE     1.0f    // 右輪比例 (調校用)

// 超時保護
#define RUN_TIMEOUT     300000  // 5 分鐘

#endif
