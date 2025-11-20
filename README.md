# 相机内参和外参标定工具包

> 完整的相机标定解决方案：内参标定、外参标定、AprilTag 识别、图像去畸变
> 
> 支持**海康威视**相机采集，优化 **Sony IMX264** (2448×2048, 6mm 镜头)

## � 快速导航

| 内容 | 说明 |
|------|------|
| 📖 [功能列表](#功能概览) | 6 个核心模块介绍 |
| 📚 [详细文档](#详细使用) | 完整的命令行参考 |
| 🎯 [应用示例](#应用示例) | C++/Python 集成示例 |
| ❓ [问题解答](#常见问题) | 常见问题和解决方案 |
| 🔬 [技术细节](#技术细节) | 算法原理和性能指标 |

---

## 功能概览

### 📦 核心模块一览

| 模块 | 功能 | 语言 | 说明 |
|------|------|------|------|
| `camera_node` | 图像采集 | C++ | ROS2 节点，支持海康威视相机实时采集 |
| `calibrate_camera` | 内参标定 | C++ | 自动检测棋盘格，计算相机内参矩阵和畸变系数 |
| `undistort_camera` | 图像去畸变 | C++ | 批量处理图像，消除镜头畸变 |
| `extrinsic_calib` | 外参标定 | C++ | 计算相机相对世界坐标系的位姿（旋转和平移） |
| `apriltag_detector` | 标签识别 | C++ | 识别 AprilTag，计算其在世界坐标系中的位姿 |
| `analyze_calibration.py` | 结果分析 | Python | 验证标定质量，支持多格式导出 |

### 完整工作流程

```
┌─────────────────────────────────────────────────────────────┐
│              相机标定 - 完整工作流程                        │
└─────────────────────────────────────────────────────────────┘

【阶段 1️⃣】内参标定（必需）
  📸 使用相机采集 30-50 张棋盘格图像
           ↓
  🔍 自动检测棋盘格角点，计算相机内参矩阵 K(3×3)
           ↓
  📊 计算 5 个畸变系数（径向 3 个，切向 2 个）
           ↓
  ✅ 验证标定质量（重投影误差），输出 YAML 文件
           ↓
  📁 结果：camera_calibration.yaml


【阶段 2️⃣】图像处理（可选）
  🖼️  批量去除图像畸变，生成校正后的图像
           ↓
  📁 结果：undistorted/ 文件夹


【阶段 3️⃣】外参标定（可选）
  📍 定义世界坐标系中的 17 个控制点
           ↓
  🖱️  手动或自动在图像中标记这些点的像素坐标
           ↓
  🎯 使用 PnP 算法计算相机的旋转矩阵 R(3×3) 和平移向量 t(3×1)
           ↓
  📊 验证外参合理性（正交性、行列式、重投影误差）
           ↓
  📁 结果：extrinsic.yaml


【阶段 4️⃣】AprilTag 识别（可选）
  🏷️  在图像中检测 AprilTag 标记
           ↓
  📐 计算标签在相机坐标系中的位姿
           ↓
  🌍 基于外参，变换到世界坐标系
           ↓
  📊 输出每个标签的位置和姿态
           ↓
  📁 结果：apriltag_result.yaml


【最终输出】
  ✅ 内参：camera_calibration.yaml（必需）
  ✅ 外参：extrinsic.yaml（可选）
  ✅ AprilTag：apriltag_result.yaml（可选）
  ✅ 去畸变图像：undistorted/（可选）
```

---

## 📚 详细使用

### 编译和安装

```bash
# 进入工作空间
cd ~/cam_intrinsic_calib

# 清理旧的编译文件（首次或遇到编译错误时）
rm -rf build install log

# 编译项目（推荐使用符号链接便于快速开发）
colcon build --symlink-install

# 或仅编译本包
colcon build --packages-select cam_intrinsic_calib --symlink-install

# 加载环境
source install/setup.bash

# 验证编译成功
ls install/cam_intrinsic_calib/lib/cam_intrinsic_calib/
```

预期输出：camera_node, calibrate_camera, undistort_camera, extrinsic_calib, apriltag_detector 等可执行文件

### 命令行工具参考

#### 1. `camera_node` - 图像采集

```bash
ros2 run cam_intrinsic_calib camera_node --ros-args \
  -p image_save_path:=/home/zzh/cam_intrinsic_calib/cam_data1120 \
  -p capture_fps:=1 \
  -p enable_auto_exposure:=true
```

**参数说明**：
- `image_save_path`：图像保存目录（默认 ./images）
- `capture_fps`：采集帧率，fps（默认 1）
- `enable_auto_exposure`：是否启用自动曝光（默认 true）
- `exposure_time`：手动曝光时间，μs（默认 20000）

#### 2. `calibrate_camera` - 内参标定

```bash
./install/cam_intrinsic_calib/lib/cam_intrinsic_calib/calibrate_camera \
  ./images \
  --square-size 20 \
  --output camera_calib.yaml \
  --display
```

**参数说明**：
- `<image_dir>`：（必需）包含标定图像的目录
- `--square-size <mm>`：棋盘格方块大小（mm），默认 20
- `--output <file>`：输出 YAML 文件名，默认 camera_calibration.yaml
- `--display`：显示检测过程（可选，用于调试）

#### 3. `analyze_calibration.py` - 结果分析

```bash
python3 analyze_calibration.py camera_calib.yaml \
  --numpy calib.npy \
  --text calib.txt
```

**参数说明**：
- `<calibration_file>`：（必需）YAML 格式的标定文件
- `--numpy <file>`：导出为 NumPy 格式（可选）
- `--text <file>`：导出为文本格式（可选）
- `--check-only`：仅检查有效性，不显示详细参数

#### 4. `undistort_camera` - 图像去畸变

```bash
./install/cam_intrinsic_calib/lib/cam_intrinsic_calib/undistort_camera \
  camera_calib.yaml \
  ./images \
  --output ./undistorted \
  --alpha 0.0 \
  --format png
```

**参数说明**：
- `<intrinsic_yaml>`：（必需）内参 YAML 文件
- `<input_dir>`：（必需）输入图像目录
- `--output <dir>`：输出目录，默认 ./undistorted
- `--alpha <0.0-1.0>`：图像缩放因子，0.0 移除黑边，1.0 保留全部，默认 0.0
- `--format <ext>`：输出格式 png/jpg/bmp，默认 png

#### 5. `extrinsic_calib` - 外参标定

```bash
./install/cam_intrinsic_calib/lib/cam_intrinsic_calib/extrinsic_calib \
  ./extrinsic/20251118_144406_106.jpg \
  camera_calib.yaml \
  --manual-mark \
  --output extrinsic.yaml
```

**参数说明**：
- `<image_file>`：（必需）包含棋盘格的图像
- `<intrinsic_yaml>`：（必需）内参 YAML 文件
- `--world-points <file>`：世界坐标控制点（可选，默认自动生成）
- `--image-points <file>`：图像坐标控制点（可选，默认交互标记）
- `--output <file>`：输出文件，默认 extrinsic.yaml

**交互标记操作**：
- 左键：标记一个点
- 右键：撤销最后一个点
- 滚轮：放大/缩小（0.25x - 4.0x）
- 中键+拖动：平移
- 'r'：重置
- 'q'/ESC：完成

#### 6. `apriltag_detector` - AprilTag 识别

```bash
./install/cam_intrinsic_calib/lib/cam_intrinsic_calib/apriltag_detector \
  /home/zzh/cam_intrinsic_calib/apriltag_pic/jpg/20251120_143432_69.jpg \
  camera_calib.yaml \
  extrinsic1.yaml \
  0.1 \
  --visualize \
  --output result.yaml
```

**参数说明**：
- `<image>`：（必需）包含 AprilTag 的图像
- `<intrinsic>`：（必需）内参 YAML 文件
- `<extrinsic>`：（必需）外参 YAML 文件
- `<tag_size>`：（必需）标签尺寸（米，如 15cm = 0.15）
- `--tag-family <f>`：标签族，41h12（推荐）/25h9/16h5
- `--visualize`：显示识别结果和坐标轴（可选）
- `--output <file>`：输出 YAML 文件，默认 apriltag_result.yaml

**重要**：tag_size 的单位是**米**，不是毫米！

---

## 🖼️ 标定图像要求

**棋盘格**：12 列 × 9 行，方块大小 20-30mm，黑白高对比度

**图像质量**：格式 BMP/JPG/PNG（推荐 BMP），分辨率 ≥1280×960，清晰无运动模糊

**采集指南**：
- 采集 20-50 张（推荐 30-40 张）
- 不同位置：中心、四角、边缘
- 不同角度：正面、俯视、仰视、侧倾
- 光照充足，整个棋盘格完整可见
- 检测成功率 ≥80%（通过 --display 查看）

### 输出文件

标定程序生成两个输出文件：

1. **camera_calibration.yaml** - OpenCV 兼容的 YAML 格式
   ```yaml
   image_width: 1280
   image_height: 960
   image_count: 30
   
   camera_matrix: !!opencv-matrix
     rows: 3
     cols: 3
     data: [1234.56, 0, 640, 0, 1234.56, 480, 0, 0, 1]
   
   distortion_coefficients: !!opencv-matrix
     rows: 5
     cols: 1
     data: [0.05, -0.1, 0, 0, 0]
   
   reprojection_error: 0.42
   ```

2. **camera_calibration.txt** - 易读的文本摘要
   ```
   相机标定结果
   =================================
   
   图像信息:
     分辨率: 1280 x 960
     标定图像数: 30 张
   
   相机矩阵 (K):
     fx = 1234.56 px
     fy = 1234.56 px
     cx = 640.00 px
     cy = 480.00 px
   
   畸变系数 (5参数):
     k1 = 0.0500
     k2 = -0.1000
     k3 = 0.0000
     p1 = 0.0000
     p2 = 0.0000
   
   精度指标:
     重投影误差 (RMS): 0.42 px
     精度评级: ⭐⭐⭐⭐⭐ 优秀
   ```

---

## 📊 标定结果说明

### 相机矩阵（内参）K

$$K = \begin{pmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{pmatrix}$$

- $f_x, f_y$：焦距（像素），一般 500-3000 px
- $c_x, c_y$：光心/主点（像素），通常接近图像中心

**本相机参考值**（Sony IMX264，2448×2048，6mm 镜头）：
- 理论焦距：1739.13 px
- 期望范围（±5%）：1652.2 - 1826.1 px

### 畸变系数（5 参数）

| 系数 | 含义 | 典型值 |
|------|------|--------|
| $k_1, k_2$ | 径向畸变 | -0.1 ~ 0.3 |
| $k_3$ | 径向畸变（高阶） | -0.01 ~ 0.01 |
| $p_1, p_2$ | 切向畸变 | -0.01 ~ 0.01 |

### 精度评估（重投影误差 RMS）

| RMS | 评级 | 应用场景 |
|-----|------|---------|
| < 0.5 px | ⭐⭐⭐⭐⭐ 优秀 | 精密测量、3D 重建 |
| 0.5-1.0 px | ⭐⭐⭐⭐ 良好 | 普通视觉应用 |
| 1.0-2.0 px | ⭐⭐⭐ 可接受 | 一般应用 |
| > 2.0 px | ⚠️ 需改进 | 重新采集和调整 |

---

## 💻 应用示例

### Python 中使用标定结果
```python
import cv2
import numpy as np

# 加载标定结果
fs = cv2.FileStorage("camera_calibration.yaml", cv2.FILE_STORAGE_READ)
K = fs.getNode("camera_matrix").mat()
dist = fs.getNode("distortion_coefficients").mat()
fs.release()

# 方法 1: 直接去畸变
image = cv2.imread("test.jpg")
undistorted = cv2.undistort(image, K, dist)

# 方法 2: 优化相机矩阵（自动裁剪黑边）
h, w = image.shape[:2]
new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, new_K, (w, h), cv2.CV_32F)
result = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)

# 外参使用：世界坐标 -> 相机坐标 -> 图像坐标
fs = cv2.FileStorage("extrinsic.yaml", cv2.FILE_STORAGE_READ)
R = fs.getNode("rotation_matrix").mat()
t = fs.getNode("translation_vector").mat()
fs.release()

world_point = np.array([[10.0], [20.0], [30.0]])  # mm
camera_point = R @ world_point + t
image_point = K @ camera_point / camera_point[2]
print(f"图像坐标: ({image_point[0, 0]}, {image_point[1, 0]})")
```

### C++ 中使用标定结果
```cpp
#include <opencv2/opencv.hpp>

cv::FileStorage fs("camera_calibration.yaml", cv::FileStorage::READ);
cv::Mat K = fs["camera_matrix"].mat();
cv::Mat dist = fs["distortion_coefficients"].mat();
fs.release();

// 去畸变
cv::Mat image = cv::imread("test.jpg");
cv::Mat undistorted;
cv::undistort(image, undistorted, K, dist);

// 使用外参进行坐标变换
cv::Mat R, t;
fs.open("extrinsic.yaml", cv::FileStorage::READ);
fs["rotation_matrix"] >> R;
fs["translation_vector"] >> t;
fs.release();
```

### 完整工作流程示例

```bash
# 第 1 步：采集 30-50 张标定图像（不同位置和角度）
ros2 run cam_intrinsic_calib camera_node --ros-args -p image_save_path:=./images

# 第 2 步：运行内参标定
./calibrate_camera ./images --square-size 20 --output camera_calib.yaml

# 第 3 步：验证标定质量（RMS 应 < 1.0 px）
python3 analyze_calibration.py camera_calib.yaml

# 第 4 步：去畸变处理（可选）
./undistort_camera camera_calib.yaml ./images --output ./undistorted

# 第 5 步：外参标定（可选，用包含棋盘格的图像）
./extrinsic_calib ./images/image_001.bmp camera_calib.yaml

# 第 6 步：AprilTag 识别（可选，若有 AprilTag）
./apriltag_detector test_image.jpg camera_calib.yaml extrinsic.yaml 0.15 --visualize
```

### 标定结果输出文件

**camera_calibration.yaml** 示例：
```yaml
image_width: 2448
image_height: 2048
image_count: 30

camera_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  data: [1739.13, 0, 1224.0, 0, 1739.13, 1024.0, 0, 0, 1]

distortion_coefficients: !!opencv-matrix
  rows: 5
  cols: 1
  data: [0.05, -0.1, 0, 0, 0]

reprojection_error: 0.42
```

**extrinsic.yaml** 示例：
```yaml
rotation_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  data: [0.998, -0.015, 0.062, 0.012, 0.999, 0.035, -0.063, -0.033, 0.998]

translation_vector: !!opencv-matrix
  rows: 3
  cols: 1
  data: [50.2, 30.5, 200.3]
```

---

---

---

## 📷 相机配置和参数

### Sony IMX264 相机规格（默认配置）

| 参数 | 值 | 说明 |
|------|-----|------|
| 分辨率 | 2448 × 2048 px | 采集分辨率 |
| 像元尺寸 | 3.45 × 3.45 μm | 像元规格 |
| 传感器宽度 | 8.4456 mm | 2448 × 3.45 / 1000 |
| 镜头焦距 | 6 mm | 标称焦距 |
| **理论焦距** | **1739.13 px** | 6 × 2448 / 8.4456 |
| **期望范围** | **1652-1826 px** | ±5% 的理想范围 |

### 修改其他相机参数

如果使用不同的相机，编辑 `src/calibrate_camera.cpp` 第 50-80 行：

```cpp
static const int CAMERA_WIDTH = 2448;        // 图像宽度（像素）
static const int CAMERA_HEIGHT = 2048;       // 图像高度（像素）
static const float LENS_FOCAL_LENGTH_MM = 6.0f;    // 镜头焦距（mm）
static const float SENSOR_WIDTH_MM = 8.4456f;      // 传感器宽度（mm）

// 棋盘格规格
static const int CHESSBOARD_COLS = 12;       // 棋盘格列数
static const int CHESSBOARD_ROWS = 9;        // 棋盘格行数
```

修改后重新编译：
```bash
colcon build --packages-select cam_intrinsic_calib --symlink-install
```

### 焦距计算公式

理论焦距 = 镜头焦距(mm) × 图像宽度(px) / 传感器宽度(mm)

$$f_{\text{px}} = \frac{F \times W_{\text{px}}}{W_{\text{mm}}}$$

**示例**：对于不同焦距的镜头
- 4mm 镜头 + 2448 px：4 × 2448 / 8.4456 = 1159.42 px
- 6mm 镜头 + 2448 px：6 × 2448 / 8.4456 = 1739.13 px（本项目）
- 8mm 镜头 + 2448 px：8 × 2448 / 8.4456 = 2318.84 px

---

### Q1：无法检测到棋盘格
- 检查棋盘格规格是否为 12×9
- 运行 `calibrate_camera --display` 查看检测过程
- 重新采集：确保高对比度、充足光照、整个棋盘格可见
- 修改 calibrate_camera.cpp 中的棋盘格参数后重新编译

### Q2：重投影误差很大（RMS > 2.0 px）
- 增加采集数量至 50 张
- 验证方块大小是否正确（用尺子测量）
- 确保采集的多样性（不同位置和角度）
- 检查棋盘格打印质量，避免变形

### Q3：焦距值看起来不对
**本相机理论焦距计算**：
$$f = \frac{6 \text{ mm} \times 2448 \text{ px}}{8.4456 \text{ mm}} = 1739.13 \text{ px}$$

期望范围（±5%）：1652.2 - 1826.1 px

验证脚本：
```python
# 标定得到的焦距
fx = 1739.13
sensor_width = 8.4456  # mm
image_width = 2448     # px
pixel_size = sensor_width / image_width
equiv_focal_length = fx * pixel_size  # mm
print(f"等效焦距: {equiv_focal_length:.2f}mm (应接近 6.0mm)")
```

### Q4：外参标定点标记不准确
- 使用 `--visualize` 标志查看标记效果
- 可以滚轮放大至 4× 以提高精度
- 使用预先设定的控制点文件（更精确）

### Q5：AprilTag 未能识别
- 确认图像中确实有 AprilTag
- 检查 tag_size 单位是否为米（0.15 = 15cm，不是毫米）
- 尝试不同的 tag-family：36h11（推荐）、25h9、16h5

### Q6：编译错误
常见错误及解决：
- `error: 'cv::Mat' was not declared` → 检查 OpenCV 头文件引入
- `undefined reference to 'cv::calibrateCamera'` → 检查 CMakeLists.txt 中的库链接
- `fatal error: MvCameraControl.h: No such file` → 仅在使用 camera_node 时需要海康 SDK

清理并重新编译：
```bash
cd ~/cam_intrinsic_calib
rm -rf build install log
colcon build --symlink-install
```

---

## 🔧 技术细节

### 标定算法

使用经典的 **Zhang's method**（张正友标定法），通过 OpenCV 的 `cv::calibrateCamera()` 实现。

**关键步骤**：
1. 棋盘格检测：`cv::findChessboardCorners()` + 亚像素精化 `cv::cornerSubPix()`
2. 内参计算：`cv::calibrateCamera()` 求解相机矩阵 K 和畸变系数
3. 精度评估：重投影误差 RMS


## 📁 项目结构

```
cam_intrinsic_calib/
├── src/                      C++ 源代码
│   ├── calibrate_camera.cpp  内参标定主程序
│   ├── camera_node.cpp       ROS2 图像采集节点
│   ├── undistort_camera.cpp  图像去畸变程序
│   ├── extrinsic_calib.cpp   外参标定程序
│   └── apriltag_detector.cpp AprilTag 识别程序
├── include/                  头文件和 SDK
├── README.md                 本文档
├── calibrate.sh              快速启动脚本
├── analyze_calibration.py    结果分析工具
├── CMakeLists.txt            编译配置
└── package.xml               ROS2 包配置
```

---

## � 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2025-11-13 | 初始发布 |
| 2.0 | 2025-11-19 | 添加 AprilTag 功能并精简文档 |

---

## 🎯 AprilTag 识别

**功能**：识别 AprilTag 标记，计算其在世界坐标系中的位置和姿态

**使用**：
```bash
./apriltag_detector <image> <intrinsic> <extrinsic> <tag_size_m> [--visualize]
```

**支持**：tag36h11 (推荐), tag25h9, tag16h5

**输出**：YAML 文件包含每个标签的世界坐标系位置和姿态

**注意**：tag_size 单位为米（如 15cm = 0.15）

---

**最后更新**：2025 年 11 月 19 日
