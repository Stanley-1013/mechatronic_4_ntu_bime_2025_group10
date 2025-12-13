// ============================================================================
// Arduino Protocol Unit Tests
// ============================================================================
// 測試 protocol.h/cpp 的封包建立、解析與驗證功能
// 使用簡單的 assert-style 測試框架
// ============================================================================

#include <stdint.h>

// ============================================================================
// 內嵌 protocol.h 定義（用於測試，避免硬體依賴）
// ============================================================================

#define CMD_START       0x01
#define CMD_STOP        0x02
#define CMD_AVOID_RED   0x03
#define CMD_SET_VACUUM  0x04
#define CMD_QUERY_STATE 0x05

#define STATE_IDLE      0x00
#define STATE_FIND_WALL 0x01
#define STATE_FORWARD   0x02
#define STATE_BACKUP    0x03
#define STATE_TURN_LEFT 0x04
#define STATE_DONE      0x05
#define STATE_ERROR     0xFF

#define PKT_HEADER_CMD       0xAA
#define PKT_FOOTER_CMD       0x55
#define PKT_HEADER_STATE     0xBB
#define PKT_FOOTER_STATE     0x66

#define PKT_STATE_LENGTH     12
#define PKT_CMD_MIN_LENGTH   5
#define PKT_CMD_MAX_LENGTH   260

#define FLAG_RED_DETECTED    0x01
#define FLAG_TARGET_REACHED  0x02
#define FLAG_VACUUM_ENABLED  0x04
#define FLAG_SENSOR_ERROR    0x08

#define VACUUM_OFF           0x00
#define VACUUM_ON            0x01

// ============================================================================
// Protocol 函數實作（內嵌用於測試）
// ============================================================================

uint8_t calc_cmd_checksum(uint8_t cmd, uint8_t payload_len, const uint8_t* payload) {
    uint8_t checksum = cmd ^ payload_len;
    if (payload != nullptr && payload_len > 0) {
        for (int i = 0; i < payload_len; i++) {
            checksum ^= payload[i];
        }
    }
    return checksum;
}

uint8_t calc_state_checksum(const uint8_t* state_packet) {
    uint8_t checksum = 0;
    for (int i = 0; i < 9; i++) {
        checksum ^= state_packet[i];
    }
    return checksum;
}

bool verify_cmd_packet(const uint8_t* packet, uint16_t length) {
    if (length < PKT_CMD_MIN_LENGTH) {
        return false;
    }

    if (length > PKT_CMD_MAX_LENGTH) {
        return false;
    }

    if (packet[0] != PKT_HEADER_CMD) {
        return false;
    }

    if (packet[length - 1] != PKT_FOOTER_CMD) {
        return false;
    }

    uint8_t payload_len = packet[2];

    if (length != 5 + payload_len) {
        return false;
    }

    uint8_t expected_checksum = calc_cmd_checksum(packet[1], payload_len, &packet[3]);
    if (packet[length - 2] != expected_checksum) {
        return false;
    }

    return true;
}

bool verify_state_packet(const uint8_t* packet, uint16_t length) {
    if (length != PKT_STATE_LENGTH) {
        return false;
    }

    if (packet[0] != PKT_HEADER_STATE) {
        return false;
    }

    if (packet[length - 1] != PKT_FOOTER_STATE) {
        return false;
    }

    uint8_t expected_checksum = calc_state_checksum(&packet[1]);
    if (packet[length - 2] != expected_checksum) {
        return false;
    }

    return true;
}

// ============================================================================
// 測試計數器
// ============================================================================

int tests_passed = 0;
int tests_failed = 0;

void test_assert(const char* test_name, bool condition) {
    if (condition) {
        Serial.print("PASS: ");
        Serial.println(test_name);
        tests_passed++;
    } else {
        Serial.print("FAIL: ");
        Serial.println(test_name);
        tests_failed++;
    }
}

// ============================================================================
// 測試案例
// ============================================================================

// 測試 1: Checksum 計算正確性 - CMD 封包
void test_cmd_checksum() {
    Serial.println("\n--- Test 1: CMD Checksum Calculation ---");

    // 測試無 payload 的指令
    uint8_t checksum1 = calc_cmd_checksum(CMD_START, 0, nullptr);
    uint8_t expected1 = CMD_START ^ 0x00;
    test_assert("CMD checksum (no payload)", checksum1 == expected1);

    // 測試有 payload 的指令
    uint8_t payload[] = {VACUUM_ON};
    uint8_t checksum2 = calc_cmd_checksum(CMD_SET_VACUUM, 1, payload);
    uint8_t expected2 = CMD_SET_VACUUM ^ 0x01 ^ VACUUM_ON;
    test_assert("CMD checksum (with payload)", checksum2 == expected2);

    // 測試多位元組 payload
    uint8_t payload3[] = {0x01, 0x02, 0x03};
    uint8_t checksum3 = calc_cmd_checksum(CMD_START, 3, payload3);
    uint8_t expected3 = CMD_START ^ 0x03 ^ 0x01 ^ 0x02 ^ 0x03;
    test_assert("CMD checksum (multi-byte payload)", checksum3 == expected3);
}

// 測試 2: Checksum 計算正確性 - State 封包
void test_state_checksum() {
    Serial.println("\n--- Test 2: State Checksum Calculation ---");

    // 建立測試資料: [STATE][CORNER][FRONT_H][FRONT_L][RIGHT_H][RIGHT_L][YAW_H][YAW_L][FLAGS]
    uint8_t state_data[] = {
        STATE_FORWARD,  // STATE
        0x03,           // CORNER_COUNT
        0x01,           // FRONT_DIST_H
        0x20,           // FRONT_DIST_L (288 cm)
        0x00,           // RIGHT_DIST_H
        0x50,           // RIGHT_DIST_L (80 cm)
        0x00,           // YAW_H
        0x5A,           // YAW_L (90 degrees)
        FLAG_VACUUM_ENABLED  // FLAGS
    };

    uint8_t checksum = calc_state_checksum(state_data);

    // 手動計算期望的 checksum
    uint8_t expected = STATE_FORWARD ^ 0x03 ^ 0x01 ^ 0x20 ^ 0x00 ^ 0x50 ^ 0x00 ^ 0x5A ^ FLAG_VACUUM_ENABLED;

    test_assert("State checksum calculation", checksum == expected);
}

// 測試 3: CMD 封包建立與驗證 - 無 Payload
void test_cmd_packet_no_payload() {
    Serial.println("\n--- Test 3: CMD Packet (No Payload) ---");

    // 建立 CMD_START 封包: [HEADER][CMD][PAYLOAD_LEN=0][CHECKSUM][FOOTER]
    uint8_t packet[5];
    packet[0] = PKT_HEADER_CMD;
    packet[1] = CMD_START;
    packet[2] = 0x00;  // payload_len = 0
    packet[3] = calc_cmd_checksum(CMD_START, 0, nullptr);
    packet[4] = PKT_FOOTER_CMD;

    bool valid = verify_cmd_packet(packet, 5);
    test_assert("CMD packet validation (no payload)", valid);
}

// 測試 4: CMD 封包建立與驗證 - 有 Payload
void test_cmd_packet_with_payload() {
    Serial.println("\n--- Test 4: CMD Packet (With Payload) ---");

    // 建立 CMD_SET_VACUUM 封包: [HEADER][CMD][PAYLOAD_LEN=1][PAYLOAD][CHECKSUM][FOOTER]
    uint8_t packet[6];
    uint8_t payload = VACUUM_ON;

    packet[0] = PKT_HEADER_CMD;
    packet[1] = CMD_SET_VACUUM;
    packet[2] = 0x01;  // payload_len = 1
    packet[3] = payload;
    packet[4] = calc_cmd_checksum(CMD_SET_VACUUM, 1, &payload);
    packet[5] = PKT_FOOTER_CMD;

    bool valid = verify_cmd_packet(packet, 6);
    test_assert("CMD packet validation (with payload)", valid);
}

// 測試 5: State 封包建立與驗證
void test_state_packet() {
    Serial.println("\n--- Test 5: State Packet Creation ---");

    // 建立完整的 State 封包
    uint8_t packet[12];
    packet[0] = PKT_HEADER_STATE;
    packet[1] = STATE_FORWARD;
    packet[2] = 0x02;  // corner_count = 2
    packet[3] = 0x01;  // front_dist_h
    packet[4] = 0x00;  // front_dist_l (256 cm)
    packet[5] = 0x00;  // right_dist_h
    packet[6] = 0x3C;  // right_dist_l (60 cm)
    packet[7] = 0x00;  // yaw_h
    packet[8] = 0x00;  // yaw_l (0 degrees)
    packet[9] = FLAG_VACUUM_ENABLED;
    packet[10] = calc_state_checksum(&packet[1]);
    packet[11] = PKT_FOOTER_STATE;

    bool valid = verify_state_packet(packet, 12);
    test_assert("State packet validation", valid);
}

// 測試 6: 邊界值測試 - YAW 負數 (使用 int16_t)
void test_yaw_negative() {
    Serial.println("\n--- Test 6: Boundary Test - Negative YAW ---");

    // 模擬 -90 度 (0xFF6A 的二補數表示)
    int16_t yaw = -90;
    uint8_t yaw_h = (yaw >> 8) & 0xFF;
    uint8_t yaw_l = yaw & 0xFF;

    // 建立 State 封包
    uint8_t packet[12];
    packet[0] = PKT_HEADER_STATE;
    packet[1] = STATE_TURN_LEFT;
    packet[2] = 0x01;  // corner_count
    packet[3] = 0x00;  // front_dist_h
    packet[4] = 0x64;  // front_dist_l (100 cm)
    packet[5] = 0x00;  // right_dist_h
    packet[6] = 0x50;  // right_dist_l (80 cm)
    packet[7] = yaw_h;
    packet[8] = yaw_l;
    packet[9] = 0x00;  // flags
    packet[10] = calc_state_checksum(&packet[1]);
    packet[11] = PKT_FOOTER_STATE;

    bool valid = verify_state_packet(packet, 12);
    test_assert("State packet with negative yaw", valid);

    // 驗證 yaw 解析正確性
    int16_t reconstructed_yaw = (int16_t)((packet[7] << 8) | packet[8]);
    test_assert("Negative yaw reconstruction", reconstructed_yaw == -90);
}

// 測試 7: 邊界值測試 - Distance 最大值
void test_distance_max() {
    Serial.println("\n--- Test 7: Boundary Test - Max Distance ---");

    // 測試最大距離 65535 cm
    uint16_t max_dist = 65535;
    uint8_t dist_h = (max_dist >> 8) & 0xFF;
    uint8_t dist_l = max_dist & 0xFF;

    // 建立 State 封包
    uint8_t packet[12];
    packet[0] = PKT_HEADER_STATE;
    packet[1] = STATE_FORWARD;
    packet[2] = 0x00;  // corner_count
    packet[3] = dist_h;  // front_dist_h
    packet[4] = dist_l;  // front_dist_l
    packet[5] = dist_h;  // right_dist_h
    packet[6] = dist_l;  // right_dist_l
    packet[7] = 0x00;  // yaw_h
    packet[8] = 0x00;  // yaw_l
    packet[9] = 0x00;  // flags
    packet[10] = calc_state_checksum(&packet[1]);
    packet[11] = PKT_FOOTER_STATE;

    bool valid = verify_state_packet(packet, 12);
    test_assert("State packet with max distance", valid);

    // 驗證距離解析正確性
    uint16_t reconstructed_front = ((uint16_t)packet[3] << 8) | packet[4];
    uint16_t reconstructed_right = ((uint16_t)packet[5] << 8) | packet[6];
    test_assert("Max distance reconstruction (front)", reconstructed_front == 65535);
    test_assert("Max distance reconstruction (right)", reconstructed_right == 65535);
}

// 測試 8: 錯誤處理 - 無效的 Header
void test_invalid_header() {
    Serial.println("\n--- Test 8: Error Handling - Invalid Header ---");

    uint8_t packet[5] = {0xFF, CMD_START, 0x00, 0x01, PKT_FOOTER_CMD};
    bool valid = verify_cmd_packet(packet, 5);
    test_assert("Reject packet with invalid header", !valid);
}

// 測試 9: 錯誤處理 - 無效的 Footer
void test_invalid_footer() {
    Serial.println("\n--- Test 9: Error Handling - Invalid Footer ---");

    uint8_t packet[5];
    packet[0] = PKT_HEADER_CMD;
    packet[1] = CMD_START;
    packet[2] = 0x00;
    packet[3] = calc_cmd_checksum(CMD_START, 0, nullptr);
    packet[4] = 0xFF;  // 錯誤的 footer

    bool valid = verify_cmd_packet(packet, 5);
    test_assert("Reject packet with invalid footer", !valid);
}

// 測試 10: 錯誤處理 - 無效的 Checksum
void test_invalid_checksum() {
    Serial.println("\n--- Test 10: Error Handling - Invalid Checksum ---");

    uint8_t packet[5];
    packet[0] = PKT_HEADER_CMD;
    packet[1] = CMD_START;
    packet[2] = 0x00;
    packet[3] = 0xFF;  // 錯誤的 checksum
    packet[4] = PKT_FOOTER_CMD;

    bool valid = verify_cmd_packet(packet, 5);
    test_assert("Reject packet with invalid checksum", !valid);
}

// 測試 11: 錯誤處理 - 無效的長度
void test_invalid_length() {
    Serial.println("\n--- Test 11: Error Handling - Invalid Length ---");

    // 長度不足
    uint8_t packet_short[4] = {PKT_HEADER_CMD, CMD_START, 0x00, PKT_FOOTER_CMD};
    bool valid1 = verify_cmd_packet(packet_short, 4);
    test_assert("Reject packet with insufficient length", !valid1);

    // State 封包長度不匹配
    uint8_t packet_state[11];  // 應為 12
    bool valid2 = verify_state_packet(packet_state, 11);
    test_assert("Reject state packet with wrong length", !valid2);
}

// 測試 12: FLAGS 位元操作
void test_flags_bits() {
    Serial.println("\n--- Test 12: FLAGS Bit Operations ---");

    uint8_t flags = 0x00;

    // 設定單一位元
    flags |= FLAG_RED_DETECTED;
    test_assert("Set FLAG_RED_DETECTED", (flags & FLAG_RED_DETECTED) != 0);

    // 設定多個位元
    flags |= FLAG_VACUUM_ENABLED;
    test_assert("Set FLAG_VACUUM_ENABLED", (flags & FLAG_VACUUM_ENABLED) != 0);
    test_assert("FLAGS still has RED_DETECTED", (flags & FLAG_RED_DETECTED) != 0);

    // 清除位元
    flags &= ~FLAG_RED_DETECTED;
    test_assert("Clear FLAG_RED_DETECTED", (flags & FLAG_RED_DETECTED) == 0);
    test_assert("FLAGS still has VACUUM_ENABLED", (flags & FLAG_VACUUM_ENABLED) != 0);
}

// ============================================================================
// 主程式
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("========================================");
    Serial.println("  Arduino Protocol Unit Tests");
    Serial.println("========================================");

    // 執行所有測試
    test_cmd_checksum();
    test_state_checksum();
    test_cmd_packet_no_payload();
    test_cmd_packet_with_payload();
    test_state_packet();
    test_yaw_negative();
    test_distance_max();
    test_invalid_header();
    test_invalid_footer();
    test_invalid_checksum();
    test_invalid_length();
    test_flags_bits();

    // 測試報告
    Serial.println("\n========================================");
    Serial.println("  Test Results");
    Serial.println("========================================");
    Serial.print("Tests Passed: ");
    Serial.println(tests_passed);
    Serial.print("Tests Failed: ");
    Serial.println(tests_failed);
    Serial.print("Total Tests:  ");
    Serial.println(tests_passed + tests_failed);

    if (tests_failed == 0) {
        Serial.println("\nALL TESTS PASSED!");
    } else {
        Serial.println("\nSOME TESTS FAILED!");
    }
    Serial.println("========================================");
}

void loop() {
    // 測試只執行一次
}
