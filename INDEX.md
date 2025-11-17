# 项目文件说明

本项目是一个**相机内参标定工具包**，用于快速准确地标定相机的内参矩阵和畸变系数。

**针对相机规格优化**: 2448×2048 分辨率，6mm 镜头

## 📂 文件结构概览

```
cam_intrinsic_calib/
│
├── 📄 README.md                     ⭐️ 【必读】完整使用文档（快速开始 + 详细说明）
│
├── 📄 CAMERA_CONFIG.md              ⭐️ 【新】相机参数配置与校验说明
│
├── 📄 QuickReference.md              海康 SDK 快速参考（仅供参考）
│
├── 🛠️ 工具脚本
│   ├── calibrate.sh                 一键启动标定（推荐使用）
│   └── analyze_calibration.py       标定结果分析验证
│
├── 💾 源代码
│   └── src/
│       ├── calibrate_camera.cpp     ⭐️ 标定计算主程序（已集成相机参数和自动校验）
│       ├── camera_node.cpp          相机图像采集节点（ROS2）
│       └── SimpleCapture.cpp        参考程序
│
├── 📂 SDK 头文件
│   └── include/cam_intrinsic_calib/
│       ├── MvCameraControl.h        海康相机控制 API
│       ├── CameraParams.h
│       ├── PixelType.h
│       └── ...（其他 SDK 文件）
│
└── 📋 配置文件
    ├── CMakeLists.txt               编译配置
    └── package.xml                  ROS2 包信息
```

## 🚀 快速开始（3 步，15 分钟）

### 1️⃣ 编译
```bash
cd ~/hikon_cam
colcon build --packages-select cam_intrinsic_calib
source install/setup.bash
```

### 2️⃣ 采集图像
```bash
# 采集 20-50 张棋盘格（12×9）图像
ros2 run cam_intrinsic_calib camera_node --ros-args \
  -p image_save_path:=/home/zzh/Pictures/hik
```

### 3️⃣ 运行标定
```bash
cd ~/hikon_cam/src/cam_intrinsic_calib

# 方法 A：使用快速脚本（推荐）
./calibrate.sh ~/Pictures/hik

# 方法 B：直接运行
./install/cam_intrinsic_calib/lib/cam_intrinsic_calib/calibrate_camera \
  ~/Pictures/hik --output camera_calib.yaml --square-size 20

# 方法 C：验证结果
python3 analyze_calibration.py camera_calibration.yaml
```

## 📖 文档说明

| 文件 | 内容 | 何时阅读 |
|------|------|--------|
| **README.md** | 完整使用文档（包含快速开始、详细说明、常见问题、技术细节）| 首次使用必读 |
| **QuickReference.md** | 海康 SDK 参考 | 开发 camera_node 时参考 |

## 🛠️ 核心工具

### `calibrate_camera.cpp` - 标定计算程序
```
输入:  20-50 张标定板图像 (BMP/JPG/PNG)
输出:  camera_calibration.yaml + camera_calibration.txt
功能:  自动检测棋盘格，计算相机内参矩阵和畸变系数
```

### `calibrate.sh` - 快速启动脚本
```
使用:  ./calibrate.sh <image_directory> [options]
特点:  自动编译检查 → 运行标定 → 验证结果
```

### `analyze_calibration.py` - 结果分析工具
```
使用:  python3 analyze_calibration.py camera_calibration.yaml
功能:  验证标定质量 → 自动诊断问题 → 导出多种格式
```

## 📊 标定结果输出

程序会生成两个文件：

1. **camera_calibration.yaml** - OpenCV 兼容格式
   - 相机矩阵 K (3×3)：焦距、光心
   - 畸变系数 (5 参数)：径向和切向畸变
   - 重投影误差（精度指标）

2. **camera_calibration.txt** - 易读文本格式
   - 同上，便于人工查看

## ⚡ 常用命令快速参考

```bash
# 编译项目
colcon build --packages-select cam_intrinsic_calib

# 采集图像
ros2 run cam_intrinsic_calib camera_node --ros-args \
  -p image_save_path:=/path/to/images

# 运行标定（推荐）
./calibrate.sh ~/Pictures/hik

# 验证结果
python3 analyze_calibration.py camera_calibration.yaml

# 显示检测过程（调试）
./install/cam_intrinsic_calib/lib/cam_intrinsic_calib/calibrate_camera \
  ~/Pictures/hik --display
```

## ⚠️ 遇到问题？

**查看 README.md 的"常见问题"部分**，包括：
- 无法检测棋盘格 → 解决方案
- 重投影误差太大 → 改进步骤
- 焦距值看起来不对 → 参考值和验证方法
- 立体标定 → 扩展方法

## 💡 关键参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `image_save_path` | 图像保存目录 | `/home/zzh/Pictures/hik` |
| `--square-size` | 棋盘格方块边长（mm） | 20 |
| `--output` | 输出 YAML 文件名 | `camera_calibration.yaml` |

## 📸 相机硬件配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 分辨率 | 2448 × 2048 px | 实际采集分辨率 |
| 镜头焦距 | 6 mm | 标称焦距 |
| 传感器尺寸 | 1/1.3" (6.4mm) | 传感器规格 |
| **理论焦距** | **≈ 2295 px** | 自动计算 |
| **合理范围** | **1960-3670 px** | 校验范围 |

## 📌 标定质量评级

| 重投影误差 | 质量评级 | 适用场景 |
|----------|---------|---------|
| < 0.5 px | ⭐⭐⭐⭐⭐ 优秀 | 精密测量、3D 重建 |
| < 1.0 px | ⭐⭐⭐⭐ 良好 | 普通视觉应用 |
| < 2.0 px | ⭐⭐⭐ 可接受 | 要求不高的应用 |

## 🔍 自动校验机制

程序运行完成后会自动进行以下校验：

### 1. 焦距范围校验
- ✓ fx 应在 1960-3670 px
- ✓ fy 应在 1638-3072 px
- ℹ️ 理论值约 2295 px

### 2. 等效焦距校验
- ✓ 应接近 6mm（镜头标称焦距）
- ✓ 误差应 < 1.5mm

### 3. 光心位置校验
- ✓ cx 应在 10%-90% 范围内
- ✓ cy 应在 10%-90% 范围内

### 4. 重投影误差校验
- ✓ RMS < 2.0px 为可接受
- ✓ RMS < 0.5px 为优秀

### 5. 畸变系数校验
- ✓ |k1|, |k2|, |k3| 通常 < 1.0
- ✓ |p1|, |p2| 通常 < 0.1

详见 **CAMERA_CONFIG.md** 了解详细的参数配置和校验逻辑。

## 🎯 下一步

1. ✅ 完成标定 → 获得 `camera_calibration.yaml`
2. ✅ 在应用中加载标定结果 → 使用矫正函数
3. ✅ 进行 3D 重建或立体视觉（可选）

**更详细的内容请参考 README.md**

---

**更新**: 2025 年 11 月 14 日
