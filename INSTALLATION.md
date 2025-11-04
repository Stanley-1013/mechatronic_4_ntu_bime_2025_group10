# Mechatronics Skill 安裝說明

## ✅ 已完成安裝

此 mechatronics skill 已成功安裝到 Claude Code！

### 📍 安裝位置

```
~/.claude/skills/mechtronic-robotics/
├── SKILL.md              (主要技能文檔，含 YAML frontmatter)
├── README.md             (技能說明)
└── references/           (參考文檔)
    ├── motor_control.md
    ├── sensor_integration.md
    └── computer_vision_slam.md
```

### 🎯 如何使用

此 skill 會**自動啟動**，無需手動調用。當你詢問以下相關問題時，Claude Code 會自動使用此技能：

**觸發關鍵詞：**
- 機器人、電控車、自走車
- Arduino、Raspberry Pi、ESP32
- L298N、馬達驅動、PWM 控制
- 感測器整合（IR、超聲波、IMU）
- 藍牙控制、MQTT、無線通訊
- OpenCV、影像處理、相機
- SLAM、自主導航、ORB-SLAM2
- ROS2、編碼器、閉環控制

**使用範例：**
```
"幫我設置 L298N 馬達驅動器連接 Arduino"
"如何用樹莓派實現藍牙遙控車？"
"怎麼整合超聲波感測器做障礙避障？"
"在 Raspberry Pi 上安裝 OpenCV 進行物體追蹤"
"設置 ORB-SLAM2 進行視覺導航"
```

### 📋 查看可用 Skills

在 Claude Code 中直接詢問：
```
"What skills are available?"
"列出所有可用的 skills"
```

### 🔄 更新 Skill

如果需要更新 skill 內容：

1. 編輯文件：
```bash
cd ~/.claude/skills/mechtronic-robotics
# 編輯 SKILL.md 或 references/ 下的文件
```

2. 重啟 Claude Code 使更改生效

### 🗑️ 移除 Skill

如果需要移除：
```bash
rm -rf ~/.claude/skills/mechtronic-robotics
```

### 📦 備份檔案

原始檔案和壓縮包保存在：
```
/home/han/claude_project/mechtronic_4/output/
├── mechtronic-robotics/          (原始 skill 目錄)
├── mechtronic-robotics.zip       (Skill Seeker 生成的壓縮包)
└── mechtronic-robotics.tar.gz    (tar 壓縮包)
```

### 🎓 Skill 內容概覽

**核心能力：**
1. 馬達控制系統（L298N、Diablo、ROS2）
2. 感測器整合（IR、超聲波、IMU、編碼器）
3. 電腦視覺（Picamera2、OpenCV、交叉編譯）
4. 自主導航（ORB-SLAM2、視覺 SLAM）
5. 無線通訊（藍牙、MQTT、RF24、WiFi）

**包含 8 個完整專案範例：**
1. L298N 直流馬達控制
2. ROS2 差動驅動機器人
3. 藍牙智慧型手機遙控車
4. 障礙避障機器人（多感測器融合）
5. ORB-SLAM2 導航機器人
6. Diablo 多馬達控制
7. OpenCV 視覺處理
8. IoT 馬達控制系統（MQTT）

**詳細技術文檔：**
- 2827 行技術文檔
- 完整接線圖和程式碼範例
- 故障排除指南
- 最佳實踐建議

### 💡 使用提示

1. **自動啟動**：不需要使用特殊命令，Claude 會根據你的問題自動使用此 skill
2. **具體描述**：提問時包含具體的硬體型號（如 L298N、HC-SR04）會獲得更精確的幫助
3. **參考文檔**：當 Claude 引用 references/ 下的文檔時，這些文檔包含更詳細的程式碼和說明
4. **多平台支援**：明確指定你使用的平台（Arduino/Raspberry Pi/ESP32）

### 📚 額外資源

如果需要上傳到 Claude.ai（而非 Claude Code）：
- 訪問：https://claude.ai/skills
- 上傳：`output/mechtronic-robotics.zip`

---

**安裝時間：** 2025-10-31
**Skill 版本：** 1.0.0
**總文檔量：** 2827 行
