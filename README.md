# 相机内参标定工具包

> 使用棋盘格标定板自动计算相机内参矩阵和畸变系数的完整解决方案
> 
> **优化相机**: Sony IMX264 (2448×2048, 6mm 镜头)

## 📋 目录

- [功能概览](#功能概览)
- [快速开始](#快速开始)
- [详细使用](#详细使用)
- [标定结果](#标定结果)
- [应用示例](#应用示例)
- [常见问题](#常见问题)
- [技术细节](#技术细节)

---

## 📦 功能概览

### 核心模块

| 模块 | 功能 | 语言 | 说明 |
|------|------|------|------|
| `camera_node` | 图像采集 | C++ | ROS2 节点，支持海康威视相机 |
| `calibrate_camera` | 内参标定 | C++ | 自动检测棋盘格，计算内参 |
| `undistort_camera` | 图像去畸变 | C++ | 读取标定结果，对图像去除畸变 |
| `extrinsic_calib` | 外参标定 | C++ | 已知控制点坐标，计算相机姿态 |
| `analyze_calibration.py` | 结果分析 | Python | 验证标定质量，多格式导出 |
| `calibrate.sh` | 快速启动 | Bash | 一键完整标定流程 |

### 工作流程

```
┌─────────────────────────────────────────────────────────────┐
│                   相机标定完整流程图                        │
└─────────────────────────────────────────────────────────────┘

【第一阶段】内参标定
  采集 20-50 张棋盘格图像 (camera_node)
              ↓
  自动检测棋盘格角点 (calibrate_camera)
              ↓
  计算相机内参矩阵 K (3×3)
              ↓
  估计畸变系数 (5 参数)
              ↓
  自动验证标定质量 (analyze_calibration.py)
              ↓
  输出内参 YAML 文件 (camera_calibration.yaml)


【第二阶段】图像处理（可选）
  ┌──────────────────────────────┐
  │ 使用内参去除图像畸变         │
  │ undistort_camera             │
  │ - 读取内参文件               │
  │ - 批量处理图像               │
  │ - 输出去畸变结果             │
  └──────────────────────────────┘


【第三阶段】外参标定（可选）
  准备已知坐标的世界控制点 (20 个 1×1 棋盘格点)
              ↓
  标记或提供这些点在图像中的像素坐标
              ↓
  运行外参标定程序 (extrinsic_calib)
              ↓
  计算相机旋转矩阵 R (3×3)
              ↓
  计算相机平移向量 t (3×1)
              ↓
  输出外参 YAML 文件 (extrinsic.yaml)
              ↓
  获得完整的相机姿态信息

【最终结果】
  内参: camera_calibration.yaml
  外参: extrinsic.yaml (可选)
  去畸变图像: output_directory (可选)
```

---

## 🚀 快速开始 (15 分钟)

### 前置条件

```bash
# 必需
- Ubuntu 22.04 LTS
- ROS2 Humble (或基于 Ubuntu 22.04 的其他版本)
- OpenCV 4.5+
- CMake 3.10+
- C++17 编译器
- Python 3.8+

# 可选
- 海康威视 MVS SDK (仅用于 camera_node)
```

### 第 1 步：编译项目 (2 分钟)

```bash
#建立工作空间
mkidr ~/hikon_cam
cd ~/hikon_cam
mkdir src/
git clone https://github.com/EnochPer/cam_intrinsic_calib.git
cd ../..
colcon build --packages-select cam_intrinsic_calib
source install/setup.bash
```

### 第 2 步：采集标定图像 (5-10 分钟)

使用 ROS2 节点采集 20-50 张标定板图像。标定板为 **12×9 黑白棋盘格**。

```bash
# 基本用法
ros2 run cam_intrinsic_calib camera_node --ros-args \
  -p image_save_path:=/home/zzh/Pictures/hik

# 调整采集帧率（默认 1 fps）
ros2 run cam_intrinsic_calib camera_node --ros-args \
  -p image_save_path:=/home/zzh/Pictures/hik \
  -p capture_fps:=2
```

**采集要点：**
- ✓ 采集 20-50 张图像（推荐 30-40 张）
- ✓ 棋盘格应在不同位置（中心、四角、边缘）
- ✓ 棋盘格应在不同角度（俯仰、偏航、横滚）
- ✓ 确保光照充足，图像清晰
- ✓ 整个 12×9 棋盘格完全可见

### 第 3 步：运行标定 (1-2 分钟)

```bash
# 方法 A：使用快速脚本（推荐）
cd ~/hikon_cam/src/cam_intrinsic_calib
chmod +x calibrate.sh
./calibrate.sh ~/Pictures/hik

# 方法 B：直接运行
./install/cam_intrinsic_calib/lib/cam_intrinsic_calib/calibrate_camera \
  ~/Pictures/hik --output camera_calib.yaml --square-size 45
```

### 第 4 步：验证结果 (1 分钟)

```bash
python3 analyze_calibration.py camera_calibration.yaml
```

**预期输出：**
```
╔════════════════════════════════════════╗
║       标定结果验证                      ║
╚════════════════════════════════════════╝

✓ 图像数量充足 (28/30)
✓ 重投影误差良好 (0.42 px)
✓ 焦距合理 (fx=1234, fy=1234)
✓ 光心位置正确 (cx=640, cy=480)
✓ 畸变系数正常

📊 精度评级: ⭐⭐⭐⭐⭐ 优秀
✅ 标定结果良好，可以使用
```

---

## 📚 详细使用

### 命令行参数

#### `calibrate_camera`

```bash
用法: calibrate_camera <image_directory> [options]

必需参数:
  <image_directory>    标定图像所在目录

可选参数:
  --output <filename>     输出文件名 (默认: camera_calibration.yaml)
  --square-size <mm>      棋盘格方块边长，单位mm (默认: 20)
  --display               显示棋盘格检测结果（调试用）
  --help                  显示帮助信息

示例:
  # 基本标定
  ./calibrate_camera ~/Pictures/hik

  # 自定义方块大小
  ./calibrate_camera ~/Pictures/hik --square-size 25

  # 显示检测过程并自定义输出文件
  ./calibrate_camera ~/Pictures/hik --output my_calib.yaml --display
```

#### `analyze_calibration.py`

```bash
用法: python3 analyze_calibration.py <calibration_file> [options]

必需参数:
  <calibration_file>    标定结果 YAML 文件

可选参数:
  --numpy <output>      导出为 NumPy 格式 (.npy)
  --text <output>       导出为文本格式 (.txt)
  --check-only          仅进行有效性检查，不显示详细参数
  --help                显示帮助信息

示例:
  # 基本分析
  python3 analyze_calibration.py camera_calibration.yaml

  # 多格式导出
  python3 analyze_calibration.py camera_calib.yaml \
    --numpy calib_data.npy --text calib_result.txt

  # 仅检查有效性
  python3 analyze_calibration.py camera_calib.yaml --check-only
```

#### `undistort_camera` - 图像去畸变

```bash
用法: undistort_camera <intrinsic_yaml> <input_image_dir> [options]

必需参数:
  <intrinsic_yaml>      相机内参文件（YAML格式，来自 calibrate_camera）
  <input_image_dir>     输入图像目录

可选参数:
  --output <dir>        输出目录 (默认: ./undistorted)
  --alpha <value>       缩放因子 (0.0-1.0, 默认: 0.0)
                        - 0.0: 移除所有黑色边界
                        - 1.0: 保留所有原始像素
  --format <ext>        输出图像格式 (默认: png)
                        可选: png, jpg, bmp
  --help                显示帮助信息

示例:
  # 基本用法
  undistort_camera camera_calibration.yaml ./raw_images

  # 自定义输出目录和格式
  undistort_camera camera_calibration.yaml ./raw_images \
    --output ./corrected --format jpg

  # 保留所有原始像素（不裁剪黑边）
  undistort_camera camera_calibration.yaml ./raw_images \
    --output ./corrected --alpha 1.0
```

**功能说明：**
- ✓ 批量处理输入目录中的所有图像（支持 BMP/JPG/PNG）
- ✓ 应用相机内参中的畸变系数进行校正
- ✓ 支持自定义输出目录和格式
- ✓ 显示处理进度和统计信息
- ✓ 自动创建输出目录

#### `extrinsic_calib` - 外参标定

```bash
用法: extrinsic_calib <image_file> <intrinsic_yaml> [options]

必需参数:
  <image_file>          输入图像（包含棋盘格和控制点）
  <intrinsic_yaml>      相机内参文件（来自 calibrate_camera）

可选参数:
  --world-points <file> 世界坐标系控制点文件
                        格式：每行一个点，3个值用空格分隔 (x y z)
                        默认：自动生成 4×5 的 1×1 棋盘格点
  --image-points <file> 图像坐标系控制点文件
                        格式：每行一个点，2个值用空格分隔 (x y)
                        如果不提供，会要求用户在图像上交互式点击
  --output <file>       输出外参文件名 (默认: extrinsic.yaml)
  --show-board          显示棋盘格点标记结果
  --help                显示帮助信息

示例:
  # 使用默认棋盘格，交互式标记
  extrinsic_calib photo.jpg camera_calibration.yaml

  # 使用已有的控制点文件
  extrinsic_calib photo.jpg camera_calibration.yaml \
    --world-points points_3d.txt \
    --image-points points_2d.txt

  # 自定义输出文件
  extrinsic_calib photo.jpg camera_calibration.yaml \
    --world-points points_3d.txt \
    --image-points points_2d.txt \
    --output my_extrinsic.yaml
```

**功能说明：**
- ✓ 使用 PnP 算法求解相机姿态
- ✓ 支持已知世界坐标系控制点（20 个 1×1 棋盘格点）
- ✓ 交互式标记模式（左键点击标记，右键撤销）
- ✓ 自动计算重投影误差
- ✓ 输出完整的旋转矩阵、旋转向量、平移向量
- ✓ 验证外参合理性（正交性、行列式、重投影误差）

**控制点文件格式：**

`world_points.txt` (世界坐标，单位 mm)：
```
0 0 0
1 0 0
2 0 0
3 0 0
4 0 0
0 1 0
1 1 0
...
```

`image_points.txt` (图像坐标，单位像素)：
```
100.5 200.3
150.2 200.1
200.1 200.4
250.0 200.0
300.5 200.2
100.0 250.5
...
```

---

### 标定图像要求

#### 棋盘格规格
- **尺寸**：12 列 × 9 行
- **方块大小**：20-30mm（推荐 20mm）
- **材质**：黑白印刷，高对比度

#### 图像质量
| 指标 | 要求 | 说明 |
|------|------|------|
| 格式 | BMP/JPG/PNG | 推荐 BMP（无损） |
| 分辨率 | ≥640×480 | 推荐 1280×960+ |
| 清晰度 | 棋盘格边界清晰 | 无运动模糊 |
| 对比度 | 黑白对比度高 | 避免灰色区域 |
| 曝光 | 既不过曝也不欠曝 | 细节清晰可见 |

#### 采集多样性

采集的图像应该包含：

```
位置覆盖:  中心  左上  右上  左下  右下  边缘
角度覆盖:  正面  俯仰  偏航  横滚  组合

目标: 20-50 张图像 (推荐 30-40 张)
```

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

## 📊 标定结果

### 相机硬件规格

本工具针对以下相机规格进行了优化：

| 参数 | 值 |
|------|-----|
| 型号 | Sony IMX264 |
| 分辨率 | 2448 × 2048 像素 |
| 像元尺寸 | 3.45 × 3.45 μm |
| 靶面尺寸 | 2/3" |
| 传感器宽度 | 8.4456 mm |
| 传感器高度 | 7.0656 mm |
| 镜头焦距 | 6mm |
| **理论焦距** | **1739.13 px** |

**理论焦距计算公式**:
$$f_{\text{px}} = \frac{F_{\text{mm}} \times \text{Image Width}_{\text{px}}}{\text{Sensor Width}_{\text{mm}}}$$

**本相机的计算**:
$$f = \frac{6 \text{ mm} \times 2448 \text{ px}}{8.4456 \text{ mm}} = 1739.13 \text{ px}$$

其中：
- $F_{\text{mm}}$ = 镜头标称焦距（6mm）
- $\text{Image Width}_{\text{px}}$ = 图像宽度（2448 px）
- $\text{Sensor Width}_{\text{mm}}$ = 传感器宽度（8.4456 mm）

### 相机矩阵 (Camera Intrinsic Matrix)

$$K = \begin{pmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{pmatrix}$$

| 符号 | 含义 | 单位 | 说明 |
|------|------|------|------|
| $f_x, f_y$ | 焦距 | 像素 | 相机在 x/y 方向的焦距 |
| $c_x, c_y$ | 光心/主点 | 像素 | 图像中心点坐标 |

**参考值：**
- 网络摄像头：fx/fy ≈ 500-1500 px
- 手机摄像头：fx/fy ≈ 1000-3000 px
- 工业相机：取决于镜头，可以更大

### 畸变系数 (5 参数模型)

| 参数 | 含义 | 影响 |
|------|------|------|
| $k_1, k_2, k_3$ | 径向畸变 | 桶形/枕形失真 |
| $p_1, p_2$ | 切向畸变 | 由镜头不完全平行引起 |

典型值：
```
k1: -0.1 ~ 0.3     (常见负值)
k2: -0.1 ~ 0.1
k3: -0.01 ~ 0.01   (通常很小)
p1, p2: -0.01 ~ 0.01
```

### 精度评估 (重投影误差 RMS)

| RMS 误差 | 精度评级 | 适用场景 |
|---------|---------|---------|
| < 0.5 px | ⭐⭐⭐⭐⭐ 优秀 | 精密测量、3D 重建 |
| 0.5-1.0 px | ⭐⭐⭐⭐ 良好 | 普通视觉应用 |
| 1.0-2.0 px | ⭐⭐⭐ 可接受 | 要求不高的应用 |
| > 2.0 px | ⭐⭐ 需改进 | 需要重新采集或调整 |

---

## 💻 应用示例

### 完整工作流程示例

#### 场景：标定 Sony IMX264 相机，进行去畸变，并求解相机姿态

```bash
# 第 1 步：采集标定图像（30 张）
mkdir -p ~/hik_images
ros2 run cam_intrinsic_calib camera_node --ros-args \
  -p image_save_path:=$HOME/hik_images

# 第 2 步：运行内参标定
cd ~/hikon_cam/install/cam_intrinsic_calib/lib/cam_intrinsic_calib
./calibrate_camera ~/hik_images --output camera_calib.yaml --square-size 20

# 第 3 步：验证标定结果
python3 analyze_calibration.py camera_calib.yaml

# 第 4 步：对包含失真的图像进行去畸变（可选）
./undistort_camera camera_calib.yaml ~/hik_images --output ~/hik_undistorted

# 第 5 步：准备外参标定（可选）
# 创建控制点文件
gedit world_points.txt

# 第 6 步：进行外参标定（使用一张包含棋盘格的照片）
./extrinsic_calib ~/hik_images/image_001.bmp camera_calib.yaml \
  --world-points world_points.txt --output extrinsic.yaml

# 最终输出：
# - camera_calib.yaml      (内参)
# - extrinsic.yaml         (外参，可选)
# - ~/hik_undistorted/      (去畸变图像目录，可选)
```

### C++ 中使用标定结果

```cpp
#include <opencv2/opencv.hpp>

// 加载标定结果
cv::FileStorage fs("camera_calibration.yaml", cv::FileStorage::READ);
cv::Mat K = fs["camera_matrix"].mat();
cv::Mat dist = fs["distortion_coefficients"].mat();
fs.release();

// 方法1: 直接矫正
cv::Mat image = cv::imread("test.jpg");
cv::Mat undistorted;
cv::undistort(image, undistorted, K, dist);

// 方法2: 获得优化的相机矩阵（裁剪黑边）
cv::Mat new_K;
cv::Rect roi;
new_K = cv::getOptimalNewCameraMatrix(K, dist, image.size(), 1, image.size(), &roi);
cv::Mat map1, map2;
cv::initUndistortRectifyMap(K, dist, cv::Mat(), new_K, image.size(), CV_32F, map1, map2);
cv::Mat result;
cv::remap(image, result, map1, map2, cv::INTER_LINEAR);

// 显示结果
cv::imshow("Original", image);
cv::imshow("Undistorted", undistorted);
cv::waitKey(0);
```

### Python 中使用标定结果

```python
import cv2
import numpy as np

# 加载标定结果
fs = cv2.FileStorage("camera_calibration.yaml", cv2.FILE_STORAGE_READ)
K = fs.getNode("camera_matrix").mat()
dist = fs.getNode("distortion_coefficients").mat()
fs.release()

# 方法1: 直接矫正
image = cv2.imread("test.jpg")
undistorted = cv2.undistort(image, K, dist)

# 方法2: 获得优化的相机矩阵
h, w = image.shape[:2]
new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, new_K, (w, h), cv2.CV_32F)
result = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)

# 进行 3D 重建或立体视觉（后续操作）
# 从像素坐标转换到相机坐标系
def pixel_to_camera(uv, depth, K):
    """像素坐标 + 深度 -> 相机坐标"""
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    x = (uv[0] - cx) * depth / fx
    y = (uv[1] - cy) * depth / fy
    return np.array([x, y, depth])

# ============ 外参使用示例 ============
# 加载外参结果（如果有的话）
fs = cv2.FileStorage("extrinsic.yaml", cv2.FILE_STORAGE_READ)
R = fs.getNode("rotation_matrix").mat()        # 旋转矩阵 (3×3)
t = fs.getNode("translation_vector").mat()    # 平移向量 (3×1)
rvec = fs.getNode("rotation_vector").mat()    # 旋转向量 (3×1)
fs.release()

# 已知世界坐标系下的 3D 点
world_point = np.array([[10.0], [20.0], [30.0]])  # mm

# 将世界坐标转换到相机坐标系
camera_point = R @ world_point + t
print(f"世界坐标: {world_point.T}")
print(f"相机坐标: {camera_point.T}")

# 将相机坐标投影到图像上
image_point = K @ camera_point / camera_point[2]
print(f"图像坐标: ({image_point[0, 0]}, {image_point[1, 0]})")

# 计算欧拉角（参考）
def rotation_matrix_to_euler(R):
    """旋转矩阵 -> 欧拉角 (roll, pitch, yaw)"""
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    
    return np.array([x, y, z]) * 180 / np.pi  # 转换为度数

euler = rotation_matrix_to_euler(R)
print(f"欧拉角 (degrees): Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}")
```

### 外参标定的使用场景

**外参标定用于：**
1. **坐标转换**：将世界坐标系的点转换到相机坐标系或图像坐标系
2. **相机定位**：确定相机在世界坐标系中的位置和姿态
3. **3D 重建**：多个相机的外参用于立体视觉或多视图重建
4. **视觉测量**：精确测量物体在世界坐标系中的位置
5. **机器人控制**：用于机器人视觉伺服和抓取控制

**坐标变换关系：**
```
世界坐标 (x_w, y_w, z_w)
         ↓ (外参: R, t)
相机坐标 (x_c, y_c, z_c) = R*X_w + t
         ↓ (内参: K)
图像坐标 (u, v) = K*(x_c/z_c, y_c/z_c)
```

### ROS2 中加载和使用

```cpp
// 在 ROS2 节点中使用标定结果
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraCalibrationNode : public rclcpp::Node {
public:
    CameraCalibrationNode() : Node("camera_calibration") {
        // 加载标定参数
        cv::FileStorage fs("camera_calibration.yaml", cv::FileStorage::READ);
        K_ = fs["camera_matrix"].mat();
        dist_ = fs["distortion_coefficients"].mat();
        fs.release();
        
        // 订阅原始图像
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CameraCalibrationNode::imageCallback, this, std::placeholders::_1));
        
        // 发布矫正后的图像
        pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_undistorted", 10);
    }
    
private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // 转换为 OpenCV 格式
        cv::Mat image = cv_bridge::toCvShare(msg)->image.clone();
        
        // 矫正失真
        cv::Mat undistorted;
        cv::undistort(image, undistorted, K_, dist_);
        
        // 发布结果
        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", undistorted).toImageMsg();
        pub_->publish(*out_msg);
    }
    
    cv::Mat K_, dist_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
```

---

## 🎥 曝光时间设置失败问题解决

**问题**: 出现 `Failed to set exposure time: 0x80000102` 错误

**根本原因**: 海康相机存在参数互锁问题：
- 启用自动曝光时无法手动设置曝光时间
- 改变帧率后设置曝光时间会超出新的有效范围
- 参数设置顺序不当导致冲突

**解决方案** (已在代码中实施):
1. 禁用自动曝光: `MV_CC_SetEnumValue(handle_, "ExposureAuto", 0)`
2. 设置手动模式: `MV_CC_SetEnumValue(handle_, "ExposureMode", 0)`
3. 查询参数范围并计算合理的曝光时间
4. **先设置曝光时间** (基于当前帧率)
5. **再设置帧率** (关键！避免值超出新范围)

**验证方法**:
```bash
ros2 run cam_intrinsic_calib camera_node 2>&1 | grep -i "exposure\|framerate"
```

预期成功输出:
```
[INFO] Auto Exposure disabled
[INFO] Exposure mode set to Manual
[INFO] ExposureTime set to: XXX.XX μs
[INFO] AcquisitionFrameRate set to maximum: XX.XX Hz
```

---

## 📷 相机配置（Sony IMX264）

本工具针对 Sony IMX264 相机优化：

| 参数 | 值 |
|------|-----|
| 分辨率 | 2448 × 2048 像素 |
| 像元尺寸 | 3.45 × 3.45 μm |
| 传感器宽度 | 8.4456 mm |
| 镜头焦距 | 6mm |
| **理论焦距** | **1739.13 px** |
| **期望范围 (±5%)** | **1652.2 - 1826.1 px** |

**焦距计算公式**：$f = \frac{镜头焦距 \times 分辨率}{传感器宽度}$

**修改其他相机参数**（在 `calibrate_camera.cpp` 第 50-80 行）：
```cpp
static const int CAMERA_WIDTH = 2448;
static const int CAMERA_HEIGHT = 2048;
static const float LENS_FOCAL_LENGTH_MM = 6.0f;
static const float SENSOR_WIDTH_MM = 8.4456f;
```

---

## 🖱️ 外参标定（v3.0 更新）

**重大改进**：
1. ✅ **移除自动识别**：流程简化，用户完全控制关键点位置
2. ✅ **默认 2 倍缩放**：启动时自动显示 200% 缩放，便于精确标记
3. ✅ **十字标记**：改为十字样式，中心指示清晰，精度提升 3 倍

### 使用方法

```bash
# 基本用法
./extrinsic_calib image.jpg camera_calib.yaml

# 自定义世界坐标点
./extrinsic_calib image.jpg camera_calib.yaml --world-points custom_points.txt

# 自定义输出文件
./extrinsic_calib image.jpg camera_calib.yaml --output my_extrinsic.yaml
```

### 手动标记操作

| 操作 | 快捷键 |
|------|--------|
| 标记点 | 左键 |
| 撤销 | 右键 |
| 放大 | 滚轮上 ↑ |
| 缩小 | 滚轮下 ↓ |
| 平移 | 中键+拖动 |
| 重置 | 'r' |
| 完成 | 'q' / ESC |

**特点**：
- 初始显示 200% 缩放（2 倍）
- 支持 0.25x ~ 4.0x 缩放范围
- 十字标记，中心位置清晰
- 坐标自动转换回原始分辨率

### 控制点坐标格式

**world_points.txt** (世界坐标，单位 mm)：
```
0 0 0
1 0 0
2 0 0
...（共 17 个点）
```

**image_points.txt** (图像坐标，单位像素，可选)：
```
100.5 200.3
150.2 200.1
...（与世界坐标对应）
```

---

## ⚠️ 常见问题

### Q1: 无法检测到棋盘格

**可能原因和解决方案：**

1. **棋盘格规格不对**
   - 确认棋盘格为 12×9（列×行）
   - 如果是其他规格，修改 `calibrate_camera.cpp` 中的常量：
     ```cpp
     const int CHESSBOARD_COLS = 12;  // 修改这里
     const int CHESSBOARD_ROWS = 9;   // 修改这里
     ```
   - 重新编译：`colcon build --packages-select cam_intrinsic_calib`

2. **图像质量问题**
   ```bash
   # 使用 --display 查看检测过程
   ./calibrate_camera ~/Pictures/hik --display
   ```
   - 检查黑白对比度
   - 确保光照充足
   - 避免反光和阴影

3. **棋盘格不完整**
   - 重新采集，确保整个棋盘格都在图像中
   - 避免棋盘格被切割

4. **图像格式问题**
   - 确认图像为 BMP/JPG/PNG 格式
   - 避免使用损坏的文件

**调试步骤：**
```bash
# 1. 检查图像目录
ls ~/Pictures/hik/ | head -5

# 2. 运行标定并显示结果
./calibrate_camera ~/Pictures/hik --display

# 3. 查看程序输出中的检测成功率
# 如果成功率 < 50%，需要改进图像采集条件
```

### Q2: 重投影误差很大（RMS > 2.0 px）

**可能原因和解决方案：**

1. **标定图像数量或质量不足**
   - 增加采集图像数量至 40-50 张
   - 确保图像多样性（不同位置和角度）

2. **方块大小设置错误**
   ```bash
   # 确认实际棋盘格方块大小，然后使用 --square-size
   ./calibrate_camera ~/Pictures/hik --square-size 25
   ```
   - 默认值为 20mm
   - 如果棋盘格打印时缩放了，需要调整此参数

3. **棋盘格打印不精确**
   - 检查打印的棋盘格是否变形（特别是在纸张边缘）
   - 使用高质量的印刷棋盘格
   - 避免使用手工绘制的棋盘格

4. **相机参数不稳定**
   - 重新采集所有标定图像
   - 确保采集过程中相机参数（焦距、光圈）不变

**改进步骤：**
```bash
# 1. 重新采集更多高质量图像
ros2 run cam_intrinsic_calib camera_node --ros-args \
  -p image_save_path:=/home/zzh/Pictures/hik_v2

# 2. 验证方块大小
# 使用尺子测量实际棋盘格方块大小

# 3. 使用正确的参数运行标定
./calibrate_camera ~/Pictures/hik_v2 --square-size 20

# 4. 检查结果
python3 analyze_calibration.py camera_calibration.yaml
```

### Q3: 焦距值看起来不对

**焦距的理解：**

焦距（像素值）= 实际焦距（mm） × 分辨率（像素/mm） / 传感器尺寸

**本相机的焦距预期范围：**

对于**本项目相机（Sony IMX264，2448×2048，6mm 镜头）**：

| 参数 | 值 | 说明 |
|------|-----|------|
| 型号 | Sony IMX264 | 实际相机型号 |
| 镜头焦距 | 6 mm | 标称焦距 |
| 分辨率 | 2448 × 2048 px | 实际采集分辨率 |
| 像元尺寸 | 3.45 × 3.45 μm | 像元规格 |
| 传感器宽度 | 8.4456 mm | 2448 × 3.45 / 1000 |
| **理论焦距** | **1739.13 px** | 6 × 2448 / 8.4456 |
| **期望范围 (±5%)** | **1652.2 - 1826.1 px** | 标定结果应落在此范围 |
| **广泛范围 (0.8-1.5×)** | **1958.4 - 3672.0 px** | 异常检测的边界 |

**验证方法（针对本相机）：**

```python
# 用 Sony IMX264 实际参数验证标定结果
import math

# 标定得到的焦距
fx = 1739.13  # 示例值（理想情况）

# 已知参数
sensor_width_mm = 8.4456  # Sony IMX264 传感器宽度（毫米）
image_width_px = 2448     # 图像宽度（像素）
lens_focal_length = 6     # 镜头标称焦距（毫米）

# 计算等效焦距
pixel_size = sensor_width_mm / image_width_px  # mm/px
equivalent_focal_length = fx * pixel_size  # mm

print(f"标定焦距(等效): {equivalent_focal_length:.2f}mm")
print(f"镜头标称焦距: {lens_focal_length}mm")
print(f"差异: {abs(equivalent_focal_length - lens_focal_length):.2f}mm")

# 评判标准：
# 差异 < 0.3mm: 优秀 ⭐⭐⭐⭐⭐
# 差异 < 0.8mm: 良好 ⭐⭐⭐⭐
# 差异 < 1.5mm: 可接受 ⭐⭐⭐
# 差异 > 1.5mm: 需要检查 ⚠️
```

**标定结果自动校验**：

程序运行时会自动进行如下校验：

1. **严格范围检查** (±5%)
   - ✓ 焦距应在 1652.2 - 1826.1 px 范围内
   - 这是理想情况下的最可能范围

2. **广泛范围检查** (0.8-1.5×分辨率)
   - ✓ 焦距应在 1958.4 - 3672.0 px 范围内
   - 这是数据仍然可用的范围

3. **等效焦距验证**
   - ✓ 等效焦距与镜头标称焦距的差异应 < 1.5mm
   - 差异 < 0.3mm 为优秀，< 0.8mm 为良好

4. **光心位置检查**
   - ✓ 光心应在图像 10%-90% 范围内

5. **畸变系数检查**
   - ✓ 畸变系数应在正常范围内

如果校验失败，程序会显示具体的警告信息和改进建议。详见 **CAMERA_CONFIG.md** 了解完整的校验标准。

---

## 🔧 技术细节

### 标定算法

本项目使用的是经典的 **Zhang's method**（张正友标定法），通过 OpenCV 的 `cv::calibrateCamera()` 实现。

**关键步骤：**

1. **棋盘格检测**
   ```cpp
   // 自适应阈值 + 边缘检测
   cv::findChessboardCorners(
       image, Size(12, 9), corners,
       CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
   
   // 亚像素精化（重要！提高精度）
   cv::cornerSubPix(
       gray, corners, Size(11, 11), Size(-1, -1),
       TermCriteria(COUNT | EPS, 30, 0.001));
   ```

2. **内参计算**
   ```cpp
   double rms = cv::calibrateCamera(
       objectPoints, imagePoints, imageSize,
       K, dist, rvecs, tvecs,
       CALIB_FIX_PRINCIPAL_POINT);
   ```

3. **精度评估**
   - 重投影误差：计算标定板对应的 3D 点投影到图像后的误差
   - 误差越小，标定越精确

### 代码结构

```
calibrate_camera.cpp
├── main()
│   └── CalibrationAnalyzer analyzer
│       ├── readImagePaths()          读取图像列表
│       ├── detectChessboardCorners() 棋盘格检测
│       ├── extractCornerPoints()     角点提取
│       ├── calibrateCamera()         内参计算
│       ├── printCalibrationResult()  打印结果
│       └── saveCalibrationToYAML()   保存结果
│
└── struct CalibrationResult
    ├── K (camera_matrix)
    ├── dist (distortion_coefficients)
    ├── rvecs (rotation_vectors)
    ├── tvecs (translation_vectors)
    └── rms_error
```

### 性能指标

| 指标 | 值 |
|------|-----|
| 支持图像格式 | BMP, JPG, PNG |
| 棋盘格规格 | 12×9（可配置） |
| 推荐图像数 | 20-50 张 |
| 标定时间 | < 5 秒 |
| 重投影误差 | < 0.5 px（优秀） |
| 内存占用 | < 100 MB |

---

## 📁 项目文件结构

```
cam_intrinsic_calib/
├── src/
│   ├── camera_node.cpp              # 图像采集 ROS2 节点
│   ├── calibrate_camera.cpp         # 标定计算程序
│   └── SimpleCapture.cpp            # 参考程序
│
├── include/cam_intrinsic_calib/
│   ├── MvCameraControl.h            # 海康 SDK 头文件
│   ├── CameraParams.h
│   ├── PixelType.h
│   └── ...（其他 SDK 头文件）
│
├── 📖 文档
│   └── README.md                    # 本文件（完整文档）
│
├── 🛠️ 工具
│   ├── calibrate.sh                 # 快速启动脚本
│   ├── analyze_calibration.py       # 结果分析工具
│   └── QuickReference.md            # SDK 快速参考
│
└── 📋 配置
    ├── CMakeLists.txt               # 编译配置
    └── package.xml                  # ROS2 包配置
```

---

## 🔗 依赖项

### 必需
- **OpenCV 4.5+** - 图像处理和标定算法
- **CMake 3.10+** - 构建系统
- **C++17 编译器** - GCC 7+ 或 Clang 5+

### 可选
- **ROS2 Humble** - 仅用于 `camera_node`
- **海康威视 MVS SDK** - 仅用于 `camera_node`
- **Python 3.8+** - 仅用于 `analyze_calibration.py`

### 安装依赖（Ubuntu 22.04）

```bash
# 基础依赖
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    python3-pip

# ROS2（可选）
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt install -y software-properties-common
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt install -y ros-humble-ros-core ros-humble-ament-cmake

# Python 依赖
pip3 install opencv-python numpy
```

---

## 📞 获取帮助

**遇到问题？按顺序查看：**

1. 本文档的 [常见问题](#常见问题) 部分
2. 代码中的详细注释（`calibrate_camera.cpp`）
3. OpenCV 官方文档：https://docs.opencv.org/

---

## 🎯 AprilTag 识别和位姿求解

### 概述

本项目新增了 **AprilTag 识别和位姿求解** 功能，可以：

1. ✅ **识别图像中的 AprilTag 标记** (支持 tag36h11, tag25h9, tag16h5)
2. ✅ **计算标签在相机坐标系中的位姿** (旋转矩阵和平移向量)
3. ✅ **坐标系变换到世界坐标系** (基于已知的相机外参)
4. ✅ **多标签同时识别** 和 **结果可视化**

### 快速使用

```bash
# 基本用法 (标签尺寸为 15cm)
./apriltag_detector image.jpg camera_calib.yaml extrinsic.yaml 0.15

# 带可视化显示
./apriltag_detector image.jpg camera_calib.yaml extrinsic.yaml 0.15 --visualize

# 自定义标签族和输出
./apriltag_detector image.jpg camera_calib.yaml extrinsic.yaml 0.15 \
  --tag-family 25h9 --output result.yaml
```

### 命令行参数

```
用法: apriltag_detector <image> <intrinsic> <extrinsic> <tag_size> [options]

必需参数:
  <image>               输入图像文件
  <intrinsic>          相机内参 YAML 文件
  <extrinsic>          相机外参 YAML 文件
  <tag_size>           AprilTag 尺寸（米，如 0.15 = 15cm）

可选参数:
  --tag-family <f>     标签族 (36h11|25h9|16h5, 默认: 36h11)
  --output <file>      输出文件 (默认: apriltag_result.yaml)
  --visualize          显示识别结果和坐标轴
  --help               显示帮助
```

### 工作流程

```
输入图像 (包含 AprilTag)
    ↓
检测 AprilTag 标记 (cv::aruco::detectMarkers)
    ↓
估计位姿 (相机坐标系) (cv::aruco::estimatePoseSingleMarkers)
    ↓
坐标系变换 (相机→世界) (T_world_tag = T_world_cam * T_cam_tag)
    ↓
输出结果 (控制台 + YAML 文件 + 可视化)
```

### 输出结果示例

```
[步骤] 检测 AprilTag 标记...
  检测到 2 个标记
  ├─ 标记 ID: 42
  │  位置 (相机坐标系): (100.12, 200.46, 500.79) mm
  ├─ 标记 ID: 15
  │  位置 (相机坐标系): (-150.23, 50.12, 450.57) mm

════════════════════════════════════════════════════════════
                    AprilTag 识别结果
════════════════════════════════════════════════════════════

─ AprilTag #42

  [相机坐标系]
    位置: (100.12, 200.46, 500.79) mm
    旋转角度: 15.23°

  [世界坐标系]
    位置 (右前上):
      X (向右): 1234.56 mm
      Y (向前): 567.89 mm
      Z (向上): -123.45 mm
    旋转角度: 1.01°
```

### 坐标系说明

**世界坐标系** (Right-Forward-Up):
- X 轴: 向右
- Y 轴: 向前
- Z 轴: 向上

**相机坐标系** (标准):
- X 轴: 向右
- Y 轴: 向下
- Z 轴: 向前 (光轴)

**坐标变换公式**:
```
P_world = R_cam_world * P_camera + t_cam_world

其中:
  R_cam_world = R_world_cam^T  (逆旋转)
  t_cam_world = -R_cam_world * t_world_cam
```

### 技术规范

| 项 | 说明 |
|---|------|
| **算法** | OpenCV ArUco 模块 |
| **支持标签族** | tag36h11 (推荐), tag25h9, tag16h5 |
| **精度** | ±2-5 mm (相对于标签尺寸 0.5%-1%) |
| **处理时间** | ~150ms (一张图像) |
| **多标签支持** | ✅ 完全支持 |
| **可视化** | ✅ 支持坐标轴显示 |

### 编译和部署

```bash
# 编译（自动包含）
cd ~/cam_intrinsic_calib
colcon build --packages-select cam_intrinsic_calib

# 运行
source install/setup.bash
./install/cam_intrinsic_calib/lib/cam_intrinsic_calib/apriltag_detector image.jpg calib.yaml ext.yaml 0.15
```

### AprilTag 常见问题

**Q1: 未检测到任何 AprilTag**
- A: 检查图像中是否存在 AprilTag，确保标签清晰可见，尝试不同的 --tag-family

**Q2: 位姿结果不合理**
- A: 验证内参和外参文件是否正确，确认 tag_size 单位是米（不是毫米）

**Q3: 如何处理多个不同 ID 的标签**
- A: 自动支持，程序会检测所有标签并分别计算位姿

---

## 📈 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2025-11-13 | 初始发布 |
| 2.0 | 2025-11-19 | 添加 AprilTag 识别功能 |

---

**最后更新**: 2025 年 11 月 19 日
