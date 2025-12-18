// v5_competition - 期末競賽最終版
// 成績：113/120 (94.2%)

// =================== 腳位設定 (確認版) ===================

// --- 繼電器 (吸塵器) ---
const int RELAY_PIN = A3; // 你的繼電器腳位

// --- 左馬達 (L298N) ---
const int L_PWM = 3;   // ENA
const int L_IN1 = 6;   // IN1
const int L_IN2 = 5;   // IN2

// --- 右馬達 (L298N) ---
const int R_PWM = 11;  // ENB
const int R_IN1 = 10;  // IN3
const int R_IN2 = 9;   // IN4

// --- 超音波腳位 ---
const int TRIG_F = 7;
const int ECHO_F = 8;

const int TRIG_RF = A1;
const int ECHO_RF = A2;

const int TRIG_RB = 2;
const int ECHO_RB = 4;

// =================== 參數設定 ===================

// 直線基準速度
int BASE_SPEED_L = 64;
int BASE_SPEED_R = 68;

// 左轉設定
int TURN_SPEED_R = 78;
int TURN_SPEED_L = 0;

// --- PID 參數 ---
float Kp = 10.0;
float Kd = 2.0;

float last_error = 0;
float last_filtered_error = 0;

// 門檻設定
const int TURN_THRESHOLD = 23; // (front distance)
const int WALL_MISSING_DIST = 30;
const float MAX_ERROR_CHANGE = 40.0;

// =================== 新增：入口只執行一次 ===================
bool entry_done = false;


void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("Vacuum ON");

  pinMode(L_PWM, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_RF, OUTPUT); pinMode(ECHO_RF, INPUT);
  pinMode(TRIG_RB, OUTPUT); pinMode(ECHO_RB, INPUT);

  stopMotor();
  Serial.println("System Ready");
  delay(250);
}


// =================== Loop 主迴圈 ===================

void loop() {

  // ====================================================
  // ⭐⭐ 入口動作（直走 1 秒 → 右轉 600 ms）只執行一次 ⭐⭐
  // ====================================================
  if (!entry_done) {
    Serial.println("ENTRY: forward + right turn");

    // Step 1: 直走 1 秒
    setMotor(64, 70);
    delay(2000);

    // Step 2: 右轉（左輪動、右輪停）
    setMotor(60, -20);
    delay(950);

    // Step 3: 直走 1 秒
    setMotor(64, 68);
    delay(1200);


    stopMotor();
    delay(200);

    entry_done = true;  // 不再重複入口動作
    last_error = 0;
    last_filtered_error = 0;
    return;
  }



  // ====================================================
  // ★ 以下原程式碼完全不動 ★
  // ====================================================

  // 監聽 RPi 訊號
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'S') {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("CMD: STOP");
    }
    if (cmd == 'O') {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("CMD: ON");
    }
  }

  float dist_F = getDistance(TRIG_F, ECHO_F);
  float dist_RF = getDistance(TRIG_RF, ECHO_RF);
  float dist_RB = getDistance(TRIG_RB, ECHO_RB);

  if (dist_F > 0 && dist_F < TURN_THRESHOLD) {
    Serial.println("Turn Left!");
    //turnLeft();
    turnLeftCombo() ;

    last_error = 0;
    last_filtered_error = 0;
  }
  else {
    if (dist_RF > 0 && dist_RB > 0) {
      PID_Control(dist_RF, dist_RB);
    } else {
      forwardBase();
    }
  }
}


// =================== PID 控制 ===================

void PID_Control(float rf, float rb) {
  float current_error = 0;

  if (rf > WALL_MISSING_DIST) {
     current_error = 2;
  }
  else {
     current_error = rf - rb;
  }

  float diff = current_error - last_filtered_error;

  if (diff > MAX_ERROR_CHANGE) {
    current_error = last_filtered_error + MAX_ERROR_CHANGE;
  }
  else if (diff < -MAX_ERROR_CHANGE) {
    current_error = last_filtered_error - MAX_ERROR_CHANGE;
  }

  last_filtered_error = current_error;

  float P = current_error * Kp;
  float D = (current_error - last_error) * Kd;
  float output = P + D;

  last_error = current_error;

  int speed_L = BASE_SPEED_L + output;
  int speed_R = BASE_SPEED_R - output;

  setMotor(speed_L, speed_R);
}


// =================== 動作函式 ===================

// void turnLeft() {
//   stopMotor();
//   delay(100);
//   setMotor(TURN_SPEED_L, TURN_SPEED_R);
//   delay(1200);
//   stopMotor();
//   delay(500);
// }

void turnLeftCombo() {

  //往右擺頭
  setMotor(TURN_SPEED_R, TURN_SPEED_L);
  delay(1100);

  //倒轉回來
  setMotor(-TURN_SPEED_R, -TURN_SPEED_L);
  delay(1300);


  //倒退、直走新家的
  //倒退
  setMotor(-70, -70);
  delay(1000);

  //直走
  setMotor(70, 70);
  delay(1000);

  // --- 動作 3：再左轉 ---
  setMotor(TURN_SPEED_L, TURN_SPEED_R);
  delay(1100);

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
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  } else {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH);
    L = abs(L);
  }
  analogWrite(L_PWM, L);

  if (R >= 0) {
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
  } else {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH);
    R = abs(R);
  }
  analogWrite(R_PWM, R);
}

void stopMotor() {
  setMotor(0, 0);
}

float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}
