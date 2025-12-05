# E2E 測試套件

## 概述

牆邊跟隨機器人的端到端測試框架，驗證自走車在多種環境和條件下的控制行為。

測試涵蓋 5 個標準場景、多維度物理效應（感測器延遲、輪速差異、控制迴圈阻塞）和 15+ 種驗證斷言。

## 快速開始

### 安裝依賴

```bash
cd /home/han/claude_project/mechtronic_4/v2_autonomous/tests/e2e
pip install -r requirements.txt
```

### 執行測試

```bash
pytest -v                    # 全部測試
pytest -m smoke -v           # 快速煙霧測試 (~3 分鐘)
pytest -m "not slow" -v      # 跳過慢速測試 (~30 分鐘)
pytest -m scenario -v        # 場景測試
pytest -m delay -v           # 感測器延遲測試
pytest -m variance -v        # 輪速差異測試
pytest -m blocking -v        # 控制迴圈阻塞測試
```

### 特定場景測試

```bash
pytest -v test_rectangular_room.py      # 矩形房間
pytest -v test_l_corridor.py             # L 型走廊
pytest -v test_sensor_delay.py           # 感測器延遲
pytest -v test_wheel_variance.py         # 輪速差異
pytest -v test_loop_blocking.py          # 迴圈阻塞
```

## 測試標記 (Markers)

| 標記 | 說明 |
|------|------|
| `smoke` | 快速驗證 (~3 分鐘) |
| `slow` | 長時間測試 |
| `scenario` | 場景測試 |
| `delay` | 感測器延遲 |
| `variance` | 輪速差異 |
| `blocking` | 控制迴圈阻塞 |
| `stress` | 壓力測試 |
| `physics` | 物理引擎測試 |
| `simulation` | 模擬器測試 |

## 標準測試場景

| 場景 | 尺寸 | 用途 | 預期行為 |
|------|------|------|--------|
| **rectangular_room** | 150×100cm | 基礎牆邊跟隨 | 4 角落, 450-500cm |
| **l_corridor** | L 型 | 轉角導航 | 4+ 角落, 300cm+ |
| **u_turn** | U 型 | U 字迴轉 | 完成迴轉, 無碰撞 |
| **narrow_corridor** | 30cm 寬 | 狹窄空間 | 無碰撞, 保持中線 |
| **open_space** | 300×300cm | 牆壁搜尋 | 尋到牆壁, 無異常停止 |

## 目錄結構

```
e2e/
├── README.md                      # 本文件
├── conftest.py                    # Pytest 配置和 fixtures
├── pytest.ini                     # Pytest 設定
├── requirements.txt               # 依賴套件列表
│
├── test_e2e_integration.py        # 完整 E2E 整合測試 (31 個測試案例)
├── test_rectangular_room.py       # 矩形房間場景測試
├── test_l_corridor.py             # L 型走廊場景測試
├── test_sensor_delay.py           # 感測器延遲測試
├── test_wheel_variance.py         # 輪速差異測試
├── test_loop_blocking.py          # 控制迴圈阻塞測試
│
├── e2e_simulator.py               # E2E 模擬器主類
├── simulation_controller.py       # 模擬器控制介面
├── test_report_generator.py       # 報告生成工具
│
├── simulator/                     # 物理模擬模組
│   ├── concrete_physics_simulator.py
│   ├── concrete_virtual_environment.py
│   └── time_controller.py
│
├── scenarios/                     # 測試場景定義
│   ├── standard_scenes.py         # 5 個標準場景
│   └── scenario_loader.py
│
├── assertions/                    # 驗證斷言模組
│   ├── e2e_assertions.py          # 15+ 種斷言
│   └── test_assertions.py
│
├── visualization/                # 軌跡視覺化
│   ├── trajectory_plotter.py
│   └── visualization_manager.py
│
├── test_results/                 # 測試報告輸出目錄
│   └── *.json                    # 測試結果 JSON 報告
│
└── QUICK_START.md, ARCHITECTURE.md, API.md, ...  # 詳細文檔
```

## 核心使用例

### 基礎模擬

```python
from e2e_simulator import E2ESimulator
from scenarios.standard_scenes import create_rectangular_room

# 建立場景和模擬器
scenario = create_rectangular_room()
simulator = E2ESimulator(
    scenario=scenario,
    sensor_delay_ms=60.0,      # 感測器延遲
    wheel_variance=0.05,        # 輪速差異 5%
    noise_level=2.0             # 感測器雜訊
)

# 執行和驗證
result = simulator.run(max_time_s=60.0)
print(f"碰撞: {result.collided}, 角落: {result.corner_count}, 距離: {result.distance_traveled:.0f}cm")
```

### 斷言驗證

```python
from assertions.e2e_assertions import (
    NoCollisionAssertion,
    MinimumCornersAssertion,
    MinimumDistanceAssertion
)

assertions = [
    NoCollisionAssertion(required=True),
    MinimumCornersAssertion(4, required=True),
    MinimumDistanceAssertion(400.0, required=False),
]

for assertion in assertions:
    result = assertion.evaluate(simulator.run(max_time_s=60.0))
    assert result.passed, assertion.failure_message
```

## CI 整合

Pytest 自動生成以下報告格式：

```bash
# HTML 報告
pytest -v --html=test_results/report.html

# JUnit XML (CI/CD)
pytest -v --junit-xml=test_results/junit.xml

# 自動結果 JSON
# 測試完成後自動存至 test_results/*.json
```

## 視覺化

使用 visualization 模組視覺化軌跡：

```python
from visualization.visualization_manager import VisualizationManager
from visualization.trajectory_plotter import TrajectoryPlotter

# 繪製軌跡
manager = VisualizationManager()
manager.plot_trajectory(result, scenario.environment)
manager.show()

# 存檔
manager.save("trajectory.png")
```

## 環境配置

所有配置在 `conftest.py` 中定義（可覆蓋）：

```python
ROBOT_RADIUS = 12.0        # cm
WHEEL_BASE = 15.0          # cm
MAX_LINEAR_SPEED = 60.0    # cm/s
SENSOR_UPDATE_FREQ = 50    # Hz
SENSOR_DELAY_MS = 50       # 預設延遲
TARGET_RIGHT_DISTANCE = 15 # 牆距目標
FRONT_STOP_DISTANCE = 15   # 前方停止距離
```

## 預期結果

### 矩形房間 (150×100cm)

- **無效應**: 4 角落, 450-500cm, <50秒
- **60ms 延遲**: 3-4 角落, 350-450cm, <70秒
- **5% 差異**: 3-4 角落, 300-400cm, <75秒
- **阻塞中**: 2+ 角落, 200cm+, <90秒

### L 型走廊

- **基礎**: 4 角落, 300cm+, <75秒
- **高壓**: 2+ 角落, 150cm+, <90秒

## 文檔導引

| 文檔 | 說明 |
|------|------|
| **README.md** | 本文件 - 快速概覽 |
| **QUICK_START.md** | 新手入門指南 |
| **RUN_TESTS.md** | 詳細測試執行 |
| **ARCHITECTURE.md** | 系統設計 |
| **API.md** | API 參考 |
| **CUSTOM_SCENARIOS.md** | 自定義場景 |
| **TROUBLESHOOTING.md** | 問題排查 |

## 常見命令速查

```bash
# 快速驗證 (~3 分鐘)
pytest -m smoke -v

# 完整測試 (~1-2 小時)
pytest -v

# 首次失敗停止
pytest -v -x

# 詳細輸出
pytest -v -s --tb=long
```

## 支援

遇到問題？參考 **TROUBLESHOOTING.md** 或查閱 **API.md** 的詳細文檔。
