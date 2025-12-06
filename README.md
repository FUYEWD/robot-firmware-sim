# 機器人控制韌體模擬平台

## 專案簡介

這是一個完整的機器人控制韌體模擬系統，展示嵌入式開發與機器人控制算法的實作能力。系統基於 QEMU 模擬 ARM Cortex-M4 微控制器，實現從底層硬體驅動到高階控制算法的完整堆疊。

## 主要功能

- **硬體抽象層**：PWM 馬達控制、編碼器讀取、IMU 感測器驅動
- **運動學計算**：正/逆運動學、雅可比矩陣計算
- **控制算法**：PID 控制器、軌跡規劃、速度控制
- **感測器融合**：卡爾曼濾波、互補濾波
- **視覺化工具**：3D 機器人模型、即時數據監控

## 專案結構

```
robot-firmware-sim/
├── firmware/           # 韌體原始碼
│   ├── src/           # 主程式碼
│   ├── inc/           # 標頭檔案
│   └── linker/        # 連結器腳本
├── simulator/         # 模擬器相關
│   ├── qemu/          # QEMU 設定檔
│   ├── visualization/ # 視覺化工具
│   └── tests/         # 模擬測試
├── tools/             # 開發工具
├── tests/             # 單元與整合測試
├── docs/              # 技術文件
└── examples/          # 範例程式
```

## 快速開始

### 環境設定

```bash
# 安裝編譯工具鏈
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi gdb-arm-none-eabi

# 安裝 QEMU 模擬器
sudo apt-get install qemu-system-arm

# 安裝 Python 相依套件
pip install -r requirements.txt
```

### 編譯專案

```bash
# 使用 Makefile 編譯
make all

# 或使用 Python 腳本
python tools/build.py
```

### 執行模擬

```bash
# 在 QEMU 中執行韌體
make run

# 啟動視覺化監控
python simulator/visualization/robot_visualizer.py

# 執行測試範例
python examples/demo_circle_trajectory.py
```

## 主要模組說明

### 1. 硬體驅動層

- **PWM 驅動**：馬達速度控制
- **編碼器驅動**：位置與速度回授
- **IMU 驅動**：慣性感測器數據讀取
- **UART 通訊**：串列通訊介面

### 2. 運動學計算

- 支援 SCARA、Delta、關節型機器人
- **正運動學**：關節角度轉換為末端位置
- **逆運動學**：末端位置轉換為關節角度
- **雅可比矩陣**：速度映射與奇異點分析

### 3. 控制算法

- **PID 控制器**：比例-積分-微分控制
- **軌跡規劃**：直線、圓弧、多項式軌跡
- **速度控制**：閉迴路速度調節
- **位置控制**：精確位置追蹤

### 4. 感測器融合

- **卡爾曼濾波**：狀態估計與雜訊濾波
- **互補濾波**：多感測器數據融合
- **姿態解算**：IMU 數據轉換為歐拉角

## API 使用範例

### 初始化機器人系統

```c
#include "robot_control.h"

int main() {
    // 機器人配置
    robot_config_t config = {
        .type = ROBOT_SCARA,
        .link_lengths = {0.2, 0.15, 0.1},
        .joint_limits = {-PI, PI, -PI/2, PI/2, 0, 0.1}
    };
    
    // 初始化系統
    robot_status_t status = robot_init(&config);
    
    if (status == ROBOT_OK) {
        printf("機器人系統初始化成功\n");
        
        // 控制機器人移動到起始位置
        joint_angles_t home = {0, 0, 0};
        robot_move_joint(&home, 2000);  // 2 秒內移動
    }
    
    return 0;
}
```

### 執行軌跡跟隨

```c
// 生成圓形軌跡
trajectory_t circle_traj;
generate_circle_trajectory(&circle_traj, 
                          center, 
                          radius, 
                          100,  // 軌跡點數
                          5000); // 總時間(ms)

// 執行軌跡
trajectory_result_t result = robot_follow_trajectory(&circle_traj);

// 檢查執行結果
if (result.success) {
    printf("軌跡執行完成\n");
    printf("最大誤差: %f mm\n", result.max_error * 1000);
    printf("平均誤差: %f mm\n", result.mean_error * 1000);
}
```

### Python 視覺化範例

```python
import robot_simulator as rs
import numpy as np

# 建立機器人模型
robot = rs.RobotArm(robot_type='scara')

# 設定軌跡
waypoints = [
    {'position': [0.1, 0.1, 0.05], 'time': 1000},
    {'position': [0.2, 0.1, 0.05], 'time': 2000},
    {'position': [0.2, 0.2, 0.05], 'time': 3000},
    {'position': [0.1, 0.2, 0.05], 'time': 4000},
    {'position': [0.1, 0.1, 0.05], 'time': 5000}
]

# 執行模擬
results = robot.execute_trajectory(waypoints)

# 顯示結果
robot.plot_trajectory(results)
robot.show_3d_animation(results)
```

## 測試範例

### 單元測試

```bash
# 執行所有單元測試
make test-unit

# 執行特定測試
make test-pid
make test-kinematics
make test-sensor-fusion
```

### 整合測試

```bash
# 執行完整系統測試
make test-integration

# 性能測試
make test-performance

# 生成測試報告
python tools/generate_test_report.py
```

## 開發指南

### 新增硬體驅動

1. 在 `firmware/src/drivers/` 建立新驅動檔案
2. 實現初始化與控制函數
3. 在 `firmware/inc/drivers/` 新增標頭檔案
4. 更新 Makefile 加入編譯選項

### 新增控制算法

1. 在 `firmware/src/algorithms/` 建立新算法
2. 實現算法核心邏輯
3. 新增對應的測試案例
4. 更新 API 文件

### 除錯與分析

```bash
# 啟動 GDB 除錯
make debug

# 性能分析
make profile

# 記憶體使用分析
make memory-usage

# 生成呼叫圖
make call-graph
```

## 性能數據

| 項目 | 數值 | 說明 |
|------|------|------|
| 運動學計算時間 | 45 µs | 正運動學計算 |
| PID 控制週期 | 12 µs | 單軸 PID 計算 |
| 感測器融合週期 | 28 µs | 卡爾曼濾波更新 |
| 總控制週期 | 850 µs | 完整控制迴圈 |
| 位置追蹤誤差 | ±0.5° | 關節角度誤差 |
| 軌跡 RMS 誤差 | 1.2 mm | 末端位置誤差 |

## 支援的硬體平台

- **模擬平台**：QEMU (STM32F4 Discovery)
- **實際硬體**：STM32F411, STM32F407, STM32F103
- **感測器**：MPU6050, AS5048A, HC-SR04
- **馬達**：步進馬達、伺服馬達、直流馬達

## 文件資源

- [API 參考手冊](docs/api-reference.md)
- [算法詳細說明](docs/algorithms.md)
- [硬體模擬指南](docs/simulation-guide.md)
- [貢獻指南](CONTRIBUTING.md)
- [故障排除](docs/troubleshooting.md)

## 實際應用案例

### 案例 1：教育訓練

本專案可用於機器人學與嵌入式系統教學，學生可在無需實際硬體的情況下學習：

- 韌體開發流程
- 控制算法實作
- 硬體/軟體協同設計

### 案例 2：算法驗證

研究人員可使用此平台驗證新的控制算法：

- 快速原型開發
- 性能比較測試
- 參數整定最佳化

### 案例 3：產品開發

工程師可在產品開發初期使用此平台：

- 架構設計驗證
- 控制邏輯測試
- 系統整合驗證

## 授權條款

本專案採用 MIT 授權條款 - 詳見 [LICENSE](LICENSE) 檔案
