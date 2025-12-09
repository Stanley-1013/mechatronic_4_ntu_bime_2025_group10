// =================== V4 Simple - 別組程式 + 紅色偵測 ===================
// 完全保留別組原始邏輯，只加入紅色偵測開關吸塵器
// 日期: 2025-12-09

// =================== 腳位設定 (確認版) ===================

// --- 左馬達 (L298N) ---
const int L_PWM = 3;   // ENA
const int L_IN1 = 6;   // IN1
const int L_IN2 = 5;   // IN2

// --- 右馬達 (L298N) ---
const int R_PWM = 11;  // ENB
const int R_IN1 = 10;  // IN3
const int R_IN2 = 9;   // IN4

// --- 超音波腳位 ---
// 1. 前方 (Front) [TRIG: 7, Echo: 8]
const int TRIG_F = 7;
const int ECHO_F = 8;

// 2. 右前 (Right Front) [TRIG: A1, Echo: A2]
const int TRIG_RF = A1;
const int ECHO_RF = A2;

// 3. 右後 (Right Back) [TRIG: D2, Echo: D4]
const int TRIG_RB = 2;
const int ECHO_RB = 4;

// --- 吸塵器繼電器 (新增) ---
const int PIN_VACUUM = A3;

// =================== 參數設定 ===================

// 直線基準速度
int BASE_SPEED_L = 64;
int BASE_SPEED_R = 77;

// 左轉設定
int TURN_SPEED_R = 70;
int TURN_SPEED_L = 0;

// --- PID 參數 ---
float Kp = 10.0;
float Kd = 2.0;

// 上一次的 Error (用於微分)
float last_error = 0;
// 上一次經過濾波的 Error (用於防突波)
float last_filtered_error = 0;

// 門檻設定
const int TURN_THRESHOLD = 20; // 前方小於 20cm 就轉彎
const int WALL_MISSING_DIST = 30; // 當右前測距大於此數值，視為「牆壁沒了/要出場了」
const float MAX_ERROR_CHANGE = 40.0; // 每次迴圈 Error 最多只能改變 40 (防止突波)

// =================== 紅色偵測 (新增) ===================
bool vacuumOn = true;  // 吸塵器狀態

// 通訊協定
const uint8_t CMD_RED_DETECTED = 0x10;   // Pi → Arduino: 偵測到紅色，關吸塵器
const uint8_t CMD_RED_CLEARED = 0x11;    // Pi → Arduino: 紅色消失，開吸塵器

// =================== Setup 初始化 ===================

void setup() {
  Serial.begin(115200);

  // 馬達腳位
  pinMode(L_PWM, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

  // 超音波腳位
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_RF, OUTPUT); pinMode(ECHO_RF, INPUT);
  pinMode(TRIG_RB, OUTPUT); pinMode(ECHO_RB, INPUT);

  // 吸塵器 (新增)
  pinMode(PIN_VACUUM, OUTPUT);
  digitalWrite(PIN_VACUUM, HIGH);  // 預設開啟

  stopMotor();
  Serial.println("System Ready: PID with Anti-Spike & Exit Logic + Red Detection");
  delay(250);
}

// =================== Loop 主迴圈 ===================

void loop() {
  // 處理 Pi 通訊 (新增)
  handleSerial();

  // 1. 讀取所有感測器
  float dist_F = getDistance(TRIG_F, ECHO_F);
  float dist_RF = getDistance(TRIG_RF, ECHO_RF);
  float dist_RB = getDistance(TRIG_RB, ECHO_RB);

  // 2. 判斷左轉 (優先權最高)
  if (dist_F > 0 && dist_F < TURN_THRESHOLD) {
    Serial.println("Turn Left!");
    turnLeft();

    last_error = 0;
    last_filtered_error = 0; // 轉彎後重置濾波紀錄
  }
  else {
    // 3. PID 沿牆控制
    // 只要有讀到數值 (排除 999 這種完全讀不到的情況，保留大數值給 PID 判斷出場)
    if (dist_RF > 0 && dist_RB > 0) {
      PID_Control(dist_RF, dist_RB);
    } else {
      forwardBase();
    }
  }
}

// =================== 通訊處理 (新增) ===================

void handleSerial() {
  while (Serial.available() >= 2) {
    uint8_t header = Serial.read();
    if (header != 0xAA) continue;  // 同步頭

    uint8_t cmd = Serial.read();

    switch (cmd) {
      case CMD_RED_DETECTED:
        // 偵測到紅色 → 關吸塵器
        digitalWrite(PIN_VACUUM, LOW);
        vacuumOn = false;
        Serial.println("RED: Vacuum OFF");
        break;

      case CMD_RED_CLEARED:
        // 紅色消失 → 開吸塵器
        digitalWrite(PIN_VACUUM, HIGH);
        vacuumOn = true;
        Serial.println("RED: Vacuum ON");
        break;
    }
  }
}

// =================== PID 運算核心 (改良版) ===================

void PID_Control(float rf, float rb) {

  float current_error = 0;

  // --- 策略 A: 出場判斷 (Exit Logic) ---
  // 如果 RF 突然變得很大 (例如 > 30cm)，代表右邊牆壁沒了
  // 這時候如果繼續算 PID，會因為 error 太大而向右急轉
  // 解決方案：強制認為 Error = 0 (直走)，或是維持極小的修正

  if (rf > WALL_MISSING_DIST) {
     // 牆壁消失，強制直走，不要右轉
     current_error = 2;
     // 如果你希望它稍微往右靠一點點去找牆，可以設成 current_error = 2; 但直走通常最安全
  }
  else {
     // 牆壁還在，正常計算平行誤差
     current_error = rf - rb;
  }

  // --- 策略 B: 梯度限制/防突波 (Gradient Clipping) ---
  // 為了避免感測器雜訊導致瞬間 Error 跳動太大 (例如突然讀到 50 又變回 5)
  // 我們限制 Error 的變化幅度不能超過 MAX_ERROR_CHANGE (例如 5)

  // 計算這次 Error 和上次濾波後 Error 的差距
  float diff = current_error - last_filtered_error;

  // 如果變化超過限制，就只允許變化一點點
  if (diff > MAX_ERROR_CHANGE) {
    current_error = last_filtered_error + MAX_ERROR_CHANGE;
  }
  else if (diff < -MAX_ERROR_CHANGE) {
    current_error = last_filtered_error - MAX_ERROR_CHANGE;
  }

  // 更新濾波後的 Error 供下次參考
  last_filtered_error = current_error;

  // --- 開始 PID 計算 (使用處理過的 current_error) ---
  float P = current_error * Kp;
  float D = (current_error - last_error) * Kd;
  float output = P + D;

  last_error = current_error;

  int speed_L = BASE_SPEED_L + output;
  int speed_R = BASE_SPEED_R - output;

  // Debug 監控
  // 這裡多印出 rf，你可以觀察出場時 rf 是否大於 30，導致 error 被強制歸 0
  Serial.print("RF:"); Serial.print(rf);
  Serial.print(" Err:"); Serial.print(current_error);
  Serial.print(" L:"); Serial.print(speed_L);
  Serial.print(" R:"); Serial.println(speed_R);

  setMotor(speed_L, speed_R);
}

// =================== 動作與感測器函式 ===================

void turnLeft() {
  stopMotor();
  delay(100);

  Serial.println("Turning Left...");
  setMotor(TURN_SPEED_L, TURN_SPEED_R);
  delay(1200);

  stopMotor();
  delay(500);
}

void forwardBase() {
  setMotor(BASE_SPEED_L, BASE_SPEED_R);
}

void setMotor(int L, int R) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  if (L >= 0) {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); analogWrite(L_PWM, L);
  } else {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); analogWrite(L_PWM, abs(L));
  }

  if (R >= 0) {
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); analogWrite(R_PWM, R);
  } else {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); analogWrite(R_PWM, abs(R));
  }
}

void stopMotor() {
  setMotor(0, 0);
}

float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 20000);

  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}
