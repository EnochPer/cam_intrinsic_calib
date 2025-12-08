/**
 * @file extrinsic_calib.cpp
 * @brief 相机外参标定程序 - 自动检测黑白角点控制点，求相机姿态
 *
 * @details
 *   该程序实现了以下功能：
 *   1. 读取相机内参文件（YAML格式）
 *   2. 读取或生成世界坐标系下的控制点坐标 (17个黑白角点)
 *   3. 从图像中自动检测黑白角点作为图像坐标系控制点
 *   4. 使用 solvePnP 计算相机的外参（旋转矩阵 R 和平移向量 t）
 *   5. 保存外参结果到 YAML 文件
 *
 * 世界坐标系：
 *   - 原点：房间墙角处，左侧为窗户，右侧为吧台，前侧为展台，后侧为墙壁
 *   - X轴正方向：向右 (Right)
 *   - Y轴正方向：向前 (Forward)
 *   - Z轴正方向：向上 (Up)
 *   - 单位：毫米 (mm)
 *
 * 控制点：
 *   - 数量：17个黑白角点
 *   - 类型：棋盘格黑白交界处的角点
 *   - 提取方法：自动角点检测算法
 *   - 位置和间距可自定义（通过世界坐标文件）
 *
 * @usage
 *   ./extrinsic_calib <image_file> <intrinsic_yaml> [options]
 *   ./extrinsic_calib <image_file> <intrinsic_yaml> --world-points
 * points_3d.txt
 *   ./extrinsic_calib <image_file> <intrinsic_yaml> --auto-detect
 *   ./extrinsic_calib <image_file> <intrinsic_yaml> --output extrinsic.yaml
 *
 * @example
 *   ./extrinsic_calib photo.jpg camera_calibration.yaml --auto-detect
 *   ./extrinsic_calib photo.jpg camera_calibration.yaml --world-points
 * points_3d.txt
 *   ./extrinsic_calib photo.jpg camera_calibration.yaml --auto-detect --output
 * extrinsic.yaml
 */

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "../include/calib_base.h"

using namespace cv;
using namespace std;

// ============================================================================
// 数据结构
// ============================================================================

/// 相机内参结构体已在 calib_base.h 中定义

/// 外参标定结果结构体
struct ExtrinsicResult {
  Mat rotationMatrix;        ///< 3×3 旋转矩阵 R
  Mat rotationVector;        ///< 3×1 旋转向量 (用于 Rodrigues 转换)
  Mat translationVector;     ///< 3×1 平移向量 t
  double reprojectionError;  ///< 重投影误差
  bool isValid;              ///< 是否有效
  string description;        ///< 描述信息

  ExtrinsicResult() : reprojectionError(0), isValid(false) {
    rotationMatrix = Mat::eye(3, 3, CV_64F);
    rotationVector = Mat::zeros(3, 1, CV_64F);
    translationVector = Mat::zeros(3, 1, CV_64F);
  }
};

// ============================================================================
// 辅助函数
// ============================================================================

/**
 * @brief 打印程序用法
 */
void printUsage(const char* programName) {
  cout << "\n════════════════════════════════════════════════════════════\n";
  cout << "             相机外参标定程序 (自动黑白角点检测)\n";
  cout << "════════════════════════════════════════════════════════════\n\n";
  cout << "用法:\n";
  cout << "  " << programName << " <image_file> <intrinsic_yaml> [options]\n\n";
  cout << "必需参数:\n";
  cout << "  <image_file>           输入图像文件（包含棋盘格）\n";
  cout << "  <intrinsic_yaml>       相机内参文件（YAML格式）\n\n";
  cout << "可选参数:\n";
  cout << "  --world-points <file>  世界坐标系控制点文件\n";
  cout
      << "                         格式：每行一个点，3个值用空格分隔 (x y z)\n";
  cout << "                         默认：自动生成 17 个标准点\n";
  cout << "  --auto-detect          自动检测图像中的黑白角点\n";
  cout << "                         （推荐方式）\n";
  cout << "  --manual-mark          交互式手动标记角点\n";
  cout << "  --output <file>        输出外参文件名 (默认: extrinsic.yaml)\n";
  cout << "  --visualize            显示检测到的角点\n";
  cout << "  --help                 显示此帮助信息\n\n";
  cout << "世界坐标系 (右前上 - Right-Forward-Up):\n";
  cout << "  X轴: 向右 (Right)\n";
  cout << "  Y轴: 向前 (Forward)\n";
  cout << "  Z轴: 向上 (Up)\n\n";
  cout << "控制点说明:\n";
  cout << "  - 类型: 黑白棋盘格角点\n";
  cout << "  - 数量: 17 个\n";
  cout << "  - 位置: 可自定义\n";
  cout << "  - 间距: 不固定\n\n";
  cout << "world_points.txt 文件格式示例 (17个点，单位mm):\n";
  cout << "  0 0 0\n";
  cout << "  10 0 0\n";
  cout << "  20 0 0\n";
  cout << "  ...\n\n";
  cout << "════════════════════════════════════════════════════════════\n\n";
}

// 注：该函数已在 calib_base.h 中声明，并在 calib_base.cpp 中实现
// CameraIntrinsics readIntrinsicsFromYAML(const string& filename);

/**
 * @brief 从文本文件读取世界坐标系控制点
 * @param filename 文本文件路径
 * @return 3D 点列表
 */
vector<Point3f> readWorldPoints(const string& filename) {
  vector<Point3f> points;
  ifstream file(filename);

  if (!file.is_open()) {
    cerr << "[ERROR] 无法打开世界坐标点文件: " << filename << endl;
    return points;
  }

  float x, y, z;
  while (file >> x >> y >> z) {
    points.push_back(Point3f(x, y, z));
  }

  file.close();

  cout << "[INFO] 读取世界坐标点: " << points.size() << " 个\n";
  return points;
}

// /**
//  * @brief 从文本文件读取图像坐标系控制点
//  * @param filename 文本文件路径
//  * @return 2D 点列表
//  */
// vector<Point2f> readImagePoints(const string& filename) {
//   vector<Point2f> points;
//   ifstream file(filename);

//   if (!file.is_open()) {
//     cerr << "[ERROR] 无法打开图像坐标点文件: " << filename << endl;
//     return points;
//   }

//   float x, y;
//   while (file >> x >> y) {
//     points.push_back(Point2f(x, y));
//   }

//   file.close();

//   cout << "[INFO] 读取图像坐标点: " << points.size() << " 个\n";
//   return points;
// }

/**
 * @brief 生成默认的世界坐标系控制点（17个黑白棋盘格角点）
 * @return 3D 点列表 (17 个点)
 *
 * 点的排列方式（棋盘格黑白角点）：
 * 世界坐标系 (Right-Forward-Up)：
 *   X 向右，Y 向前，Z 向上
 *   取中间 4×3 = 12 个，加周边 5 个 = 17 个
 */
vector<Point3f> generateDefaultWorldPoints() {
  vector<Point3f> points;

  // 生成 17 个离散的控制点，间距不等距
  // 这些点不遵循棋盘格规律，而是按照实际需求分布
  // 单位: mm，坐标系: 右(X) 前(Y) 上(Z)

  // 定义 17 个点的坐标（间距不等）
  float controlPoints[17][3] = {
      {100, 100, 0},   {1000, 100, 0},  {1500, 100, 0},  {2000, 100, 0},
      {100, 900, 0},   {1000, 900, 0},  {1500, 900, 0},  {2000, 900, 0},
      {100, 2000, 0},  {1000, 2000, 0}, {1500, 2000, 0}, {2000, 2000, 0},
      {100, 2700, 0},  {100, 3000, 0},  {1000, 3000, 0}, {1500, 3000, 0},
      {2000, 3000, 0},
  };

  // 将这些点添加到向量中
  for (int i = 0; i < 17; ++i) {
    points.push_back(
        Point3f(controlPoints[i][0], controlPoints[i][1], controlPoints[i][2]));
  }

  return points;
}

/**
 * @brief 自动检测图像中的 2×2 棋盘格中心点
 * @param image 输入图像
 * @param cornerPoints 输出的棋盘格中心点列表
 * @param targetCornerCount 目标检测点数 (默认 17)
 * @param visualize 是否可视化结果
 * @return 检测到的点数
 *
 * 算法步骤：
 *   1. 检测棋盘格的所有角点 (使用 findChessboardCorners)
 *   2. 计算相邻 2×2 方块的中心作为控制点
 *   3. 从所有中心点中选择最均匀分布的 17 个点
 */
int detectCornerPointsInImage(const Mat& image, vector<Point2f>& cornerPoints,
                              int targetCornerCount = 17,
                              bool visualize = false) {
  cout << "[步骤] 自动检测图像中的 2×2 棋盘格中心点...\n";
  cout << "════════════════════════════════════════════════════════════\n";

  cornerPoints.clear();

  // 转换为灰度图像
  Mat gray;
  if (image.channels() == 3) {
    cvtColor(image, gray, COLOR_BGR2GRAY);
  } else {
    gray = image.clone();
  }

  cout << "  [1] 灰度图像转换 ✓\n";

  // 尝试多种棋盘格规格来检测角点
  vector<Point2f> boardCorners;
  Size boardSize(12, 9);  // 标准棋盘格规格
  bool found = false;

  cout << "  [2] 检测棋盘格角点...\n";
  for (int cols = 10; cols <= 14 && !found; ++cols) {
    for (int rows = 7; rows <= 11 && !found; ++rows) {
      found = findChessboardCorners(
          gray, Size(cols, rows), boardCorners,
          CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
      if (found) {
        cout << "    找到 " << cols << "×" << rows << " 棋盘格\n";
        boardSize = Size(cols, rows);
      }
    }
  }

  if (!found) {
    cout << "  [警告] 未找到棋盘格，尝试使用 Harris 角点检测...\n";

    // 备用方案：使用 Harris 角点检测
    vector<Point2f> tempCorners;
    int maxCorners = targetCornerCount * 3;
    double qualityLevel = 0.01;
    double minDistance = 15.0;

    goodFeaturesToTrack(gray, tempCorners, maxCorners, qualityLevel,
                        minDistance, Mat(), 5, true, 0.04);

    cout << "    检测到 " << tempCorners.size() << " 个角点\n";

    if (tempCorners.size() >= targetCornerCount) {
      cornerPoints.assign(tempCorners.begin(),
                          tempCorners.begin() + targetCornerCount);
    } else {
      cornerPoints = tempCorners;
    }
  } else {
    // 从棋盘格角点计算 2×2 方块的中心
    cout << "  [3] 计算 2×2 棋盘格中心点...\n";

    vector<Point2f> centerPoints;
    int cols = boardSize.width;
    int rows = boardSize.height;

    // 棋盘格有 (cols)×(rows) 个角点
    // 可以形成 (cols-1)×(rows-1) 个 1×1 方块
    // 选择 (cols-2)×(rows-2) 个完整的 2×2 方块

    for (int i = 0; i < rows - 2; ++i) {
      for (int j = 0; j < cols - 2; ++j) {
        // 计算 2×2 方块的四个角
        Point2f p0 = boardCorners[i * cols + j];              // 左上
        Point2f p1 = boardCorners[i * cols + (j + 2)];        // 右上
        Point2f p2 = boardCorners[(i + 2) * cols + j];        // 左下
        Point2f p3 = boardCorners[(i + 2) * cols + (j + 2)];  // 右下

        // 计算中心
        Point2f center = (p0 + p1 + p2 + p3) * 0.25f;
        centerPoints.push_back(center);
      }
    }

    cout << "    计算出 " << centerPoints.size() << " 个中心点\n";

    // 选择目标数量的点（均匀分布）
    if (centerPoints.size() >= targetCornerCount) {
      // 均匀采样
      int step = centerPoints.size() / targetCornerCount;
      for (int i = 0; i < targetCornerCount && i * step < centerPoints.size();
           ++i) {
        cornerPoints.push_back(centerPoints[i * step]);
      }
      cout << "  [4] 均匀选择 " << cornerPoints.size() << " 个中心点 ✓\n";
    } else {
      cornerPoints = centerPoints;
      cout << "  [4] 使用全部 " << cornerPoints.size() << " 个中心点\n";
    }

    // 亚像素精化
    if (!cornerPoints.empty()) {
      cornerSubPix(
          gray, cornerPoints, Size(11, 11), Size(-1, -1),
          TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 20, 0.03));
      cout << "  [5] 亚像素精化 ✓\n";
    }
  }

  // 可视化
  if (visualize && cornerPoints.size() > 0) {
    Mat displayImage = image.clone();

    for (size_t i = 0; i < cornerPoints.size(); ++i) {
      circle(displayImage, cornerPoints[i], 8, Scalar(0, 255, 0), 2);
      circle(displayImage, cornerPoints[i], 4, Scalar(0, 255, 0), -1);
      putText(displayImage, to_string(i + 1),
              Point(cornerPoints[i].x + 12, cornerPoints[i].y - 8),
              FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
    }

    namedWindow("检测到的 2×2 棋盘格中心点", WINDOW_NORMAL);
    imshow("检测到的 2×2 棋盘格中心点", displayImage);
    cout << "  [6] 显示检测结果 (按任意键继续)\n";
    waitKey(0);
    destroyWindow("检测到的 2×2 棋盘格中心点");
  } else {
    cout << "  [6] 跳过可视化\n";
  }

  cout << "════════════════════════════════════════════════════════════\n";
  cout << "[完成] 检测到 " << cornerPoints.size() << " 个中心点\n\n";

  return cornerPoints.size();
}

/**
 * @brief 交互式手动标记控制点（支持图像缩放和平移）
 * @param image 输入图像
 * @param points 输出的图像坐标点列表
 * @param worldPointCount 需要标记的点数
 *
 * 改进的缩放和平移实现
 */
class PointMarker {
 public:
  vector<Point2f> points;  // 标记点（原始坐标）
  Mat originalImage;       // 原始图像

  int pointCount;
  int markedCount;

  // 缩放和平移参数
  float scale;     // 缩放比例
  int panX, panY;  // 平移

  // 交互状态
  bool isDragging;
  int dragStartX, dragStartY;

  PointMarker(const Mat& img, int count)
      : originalImage(img.clone()),
        pointCount(count),
        markedCount(0),
        scale(2.0f),
        panX(0),
        panY(0),
        isDragging(false),
        dragStartX(0),
        dragStartY(0) {}

  /**
   * @brief 生成显示图像（应用缩放和平移）
   */
  Mat renderDisplay() {
    // 创建缩放图像
    Mat scaled;
    if (scale != 1.0f) {
      int w = (int)(originalImage.cols * scale);
      int h = (int)(originalImage.rows * scale);
      resize(originalImage, scaled, Size(w, h), 0, 0, INTER_LINEAR);
    } else {
      scaled = originalImage.clone();
    }

    // 创建显示画布
    int canvasW = originalImage.cols;
    int canvasH = originalImage.rows;
    Mat display = Mat::zeros(canvasH, canvasW, originalImage.type());

    // 计算源和目标的交集
    int srcX1 = max(0, -panX);
    int srcY1 = max(0, -panY);
    int dstX1 = max(0, panX);
    int dstY1 = max(0, panY);

    int srcX2 = min(scaled.cols, canvasW - panX);
    int srcY2 = min(scaled.rows, canvasH - panY);
    int dstX2 = min(canvasW, scaled.cols + panX);
    int dstY2 = min(canvasH, scaled.rows + panY);

    int w = srcX2 - srcX1;
    int h = srcY2 - srcY1;

    if (w > 0 && h > 0) {
      scaled(Rect(srcX1, srcY1, w, h))
          .copyTo(display(Rect(dstX1, dstY1, w, h)));
    }

    // 绘制标记的点 - 十字样式
    for (int i = 0; i < markedCount; ++i) {
      int px = (int)(points[i].x * scale) + panX;
      int py = (int)(points[i].y * scale) + panY;

      if (px >= 0 && px < canvasW && py >= 0 && py < canvasH) {
        // 绘制十字
        int size = 15;
        line(display, Point(px - size, py), Point(px + size, py),
             Scalar(0, 255, 0), 2);
        line(display, Point(px, py - size), Point(px, py + size),
             Scalar(0, 255, 0), 2);

        // 中心点
        circle(display, Point(px, py), 2, Scalar(0, 255, 0), -1);

        // 点编号
        putText(display, to_string(i + 1), Point(px + 20, py - 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
      }
    }

    // 显示信息
    string info1 = "Zoom: " + to_string((int)(scale * 100)) + "%";
    putText(display, info1, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7,
            Scalar(255, 255, 0), 2);

    string info2 =
        "Points: " + to_string(markedCount) + "/" + to_string(pointCount);
    putText(display, info2, Point(10, 70), FONT_HERSHEY_SIMPLEX, 0.7,
            Scalar(255, 255, 0), 2);

    return display;
  }

  /**
   * @brief 显示坐标转原始坐标
   */
  Point2f screenToImage(int sx, int sy) {
    float ix = (sx - panX) / scale;
    float iy = (sy - panY) / scale;

    ix = max(0.0f, min((float)(originalImage.cols - 1), ix));
    iy = max(0.0f, min((float)(originalImage.rows - 1), iy));

    return Point2f(ix, iy);
  }

  static void mouseCallback(int event, int x, int y, int flags,
                            void* userdata) {
    PointMarker* self = (PointMarker*)userdata;

    if (event == EVENT_LBUTTONDOWN) {
      if (self->markedCount < self->pointCount) {
        Point2f p = self->screenToImage(x, y);
        self->points.push_back(p);
        self->markedCount++;

        cout << "[标记 " << self->markedCount << "/" << self->pointCount
             << "] 坐标: (" << fixed << setprecision(1) << p.x << ", " << p.y
             << ")\n";

        Mat disp = self->renderDisplay();
        imshow("标记点", disp);

        if (self->markedCount == self->pointCount) {
          cout << "[完成] 所有点已标记\n";
        }
      }
    } else if (event == EVENT_RBUTTONDOWN) {
      if (self->markedCount > 0) {
        self->points.pop_back();
        self->markedCount--;

        cout << "[撤销] 剩余 " << self->markedCount << " 个点\n";
        Mat disp = self->renderDisplay();
        imshow("标记点", disp);
      }
    } else if (event == EVENT_MOUSEWHEEL) {
      float oldScale = self->scale;

      if (flags > 0) {
        self->scale *= 1.15f;
        if (self->scale > 4.0f) self->scale = 4.0f;
      } else {
        self->scale /= 1.15f;
        if (self->scale < 0.25f) self->scale = 0.25f;
      }

      if (self->scale != oldScale) {
        // 保持鼠标位置指向同一原始点
        float imgX = (x - self->panX) / oldScale;
        float imgY = (y - self->panY) / oldScale;
        self->panX = x - (int)(imgX * self->scale);
        self->panY = y - (int)(imgY * self->scale);

        Mat disp = self->renderDisplay();
        imshow("标记点", disp);
      }
    } else if (event == EVENT_MBUTTONDOWN) {
      self->isDragging = true;
      self->dragStartX = x;
      self->dragStartY = y;
    } else if (event == EVENT_MOUSEMOVE && self->isDragging) {
      int dx = x - self->dragStartX;
      int dy = y - self->dragStartY;

      self->panX += dx;
      self->panY += dy;

      self->dragStartX = x;
      self->dragStartY = y;

      Mat disp = self->renderDisplay();
      imshow("标记点", disp);
    } else if (event == EVENT_MBUTTONUP) {
      self->isDragging = false;
    }
  }

  void run() {
    namedWindow("标记点", WINDOW_NORMAL);
    setMouseCallback("标记点", mouseCallback, this);

    cout << "\n════════════════════════════════════════════════════════════\n";
    cout << "                   手动标记控制点\n";
    cout << "════════════════════════════════════════════════════════════\n";
    cout << "操作说明:\n";
    cout << "  左键      → 标记点\n";
    cout << "  右键      → 撤销最后一个点\n";
    cout << "  滚轮      → 放大/缩小\n";
    cout << "  中键拖动  → 移动图像\n";
    cout << "  'r'       → 重置视图\n";
    cout << "  'q'/'ESC' → 完成标记\n";
    cout << "需要标记: " << pointCount << " 个点\n";
    cout << "════════════════════════════════════════════════════════════\n\n";

    Mat disp = renderDisplay();
    imshow("标记点", disp);

    while (markedCount < pointCount) {
      int key = waitKey(0);

      if (key == 'q' || key == 27) {
        if (markedCount == pointCount) {
          break;
        } else {
          cout << "[提示] 还需标记 " << (pointCount - markedCount) << " 个点\n";
        }
      } else if (key == 'r') {
        scale = 1.0f;
        panX = 0;
        panY = 0;

        Mat disp = renderDisplay();
        imshow("标记点", disp);
        cout << "[操作] 已重置视图\n";
      }
    }

    destroyWindow("标记点");
  }
};

/**
 * @brief 使用 solvePnP 计算相机外参
 * @param objectPoints 世界坐标系控制点
 * @param imagePoints 图像坐标系控制点
 * @param cameraMatrix 相机矩阵
 * @param distCoeffs 畸变系数
 * @return 外参标定结果
 */
ExtrinsicResult solveExtrinsic(const vector<Point3f>& objectPoints,
                               const vector<Point2f>& imagePoints,
                               const Mat& cameraMatrix, const Mat& distCoeffs) {
  ExtrinsicResult result;

  if (objectPoints.size() != imagePoints.size()) {
    cerr << "[ERROR] 世界坐标点数 (" << objectPoints.size()
         << ") 与图像坐标点数 (" << imagePoints.size() << ") 不匹配\n";
    return result;
  }

  if (objectPoints.size() < 4) {
    cerr << "[ERROR] 控制点数过少 (至少需要 4 个点)\n";
    return result;
  }

  cout << "\n[步骤] 计算相机外参 (solvePnP)...\n";
  cout << "════════════════════════════════════════════════════════════\n";
  cout << "  控制点数: " << objectPoints.size() << " 个\n";
  cout << "  求解方法: SOLVEPNP_ITERATIVE (迭代法，精度最高)\n";
  cout << "  世界坐标系: 右前上 (Right-Forward-Up)\n";

  // 使用 solvePnP 求解
  bool success = solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
                          result.rotationVector, result.translationVector,
                          false, SOLVEPNP_ITERATIVE);

  if (!success) {
    cerr << "[ERROR] solvePnP 求解失败\n";
    return result;
  }

  // 将旋转向量转换为旋转矩阵
  Rodrigues(result.rotationVector, result.rotationMatrix);

  // 计算重投影误差
  vector<Point2f> projectedPoints;
  projectPoints(objectPoints, result.rotationVector, result.translationVector,
                cameraMatrix, distCoeffs, projectedPoints);

  double totalError = 0.0;
  for (size_t i = 0; i < imagePoints.size(); ++i) {
    double dx = imagePoints[i].x - projectedPoints[i].x;
    double dy = imagePoints[i].y - projectedPoints[i].y;
    totalError += sqrt(dx * dx + dy * dy);
  }
  result.reprojectionError = totalError / imagePoints.size();

  result.isValid = true;
  cout << "[完成] 外参计算成功\n";
  cout << "════════════════════════════════════════════════════════════\n\n";

  return result;
}

/**
 * @brief 将外参结果保存为 YAML 文件
 * @param filename 输出文件名
 * @param result 外参标定结果
 */
void saveExtrinsicToYAML(const string& filename,
                         const ExtrinsicResult& result) {
  cout << "[步骤] 保存外参结果到文件: " << filename << "\n";

  FileStorage fs(filename, FileStorage::WRITE);

  if (!fs.isOpened()) {
    cerr << "[ERROR] 无法打开文件进行写入: " << filename << endl;
    return;
  }

  // 写入旋转矩阵
  fs << "rotation_matrix" << result.rotationMatrix;

  // 写入旋转向量 (Rodrigues 表示)
  fs << "rotation_vector" << result.rotationVector;

  // 写入平移向量
  fs << "translation_vector" << result.translationVector;

  // 写入重投影误差
  fs << "reprojection_error" << result.reprojectionError;

  fs.release();

  cout << "[成功] 外参结果已保存\n\n";
}

/**
 * @brief 打印外参结果
 */
void printExtrinsicResult(const ExtrinsicResult& result) {
  cout << "\n════════════════════════════════════════════════════════════\n";
  cout << "                   外参标定结果\n";
  cout << "════════════════════════════════════════════════════════════\n\n";

  cout << "─ 旋转矩阵 (Rotation Matrix) R\n";
  cout << "  单位: 无量纲\n";
  cout << "  形式: 3×3 矩阵\n";
  cout << "  说明: 世界坐标系到相机坐标系的旋转\n\n";
  cout << result.rotationMatrix << "\n\n";

  cout << "─ 旋转向量 (Rotation Vector)\n";
  cout << "  单位: 弧度\n";
  cout << "  形式: 3×1 向量 (Rodrigues 表示)\n";
  cout << "  说明: 向量方向为旋转轴，向量长度为旋转角度\n\n";
  cout << result.rotationVector << "\n\n";

  // 从旋转向量计算欧拉角（仅供参考）
  double angle = norm(result.rotationVector);
  double angleDeg = angle * 180.0 / M_PI;
  cout << "  旋转角度: " << fixed << setprecision(4) << angleDeg << "° ("
       << angle << " 弧度)\n\n";

  cout << "─ 平移向量 (Translation Vector) t\n";
  cout << "  单位: 毫米 (mm)\n";
  cout << "  形式: 3×1 向量\n";
  cout << "  说明: 世界坐标系原点在相机坐标系中的位置\n";
  cout << "        世界坐标系为 右(X) 前(Y) 上(Z)\n\n";
  cout << "  t_x (向右) = " << fixed << setprecision(2)
       << result.translationVector.at<double>(0, 0) << " mm\n";
  cout << "  t_y (向前) = " << result.translationVector.at<double>(1, 0)
       << " mm\n";
  cout << "  t_z (向上) = " << result.translationVector.at<double>(2, 0)
       << " mm\n\n";

  cout << "─ 标定精度\n";
  cout << "  重投影误差 (RMS): " << fixed << setprecision(4)
       << result.reprojectionError << " px\n";
  cout << "  评价标准:\n";
  cout << "    < 0.5 px: ⭐⭐⭐⭐⭐ 优秀\n";
  cout << "    < 1.0 px: ⭐⭐⭐⭐ 良好\n";
  cout << "    < 2.0 px: ⭐⭐⭐ 可接受\n";
  cout << "    ≥ 2.0 px: ⭐⭐ 需要改进\n\n";

  cout << "════════════════════════════════════════════════════════════\n\n";
}

/**
 * @brief 验证外参结果
 */
void validateExtrinsicResult(const ExtrinsicResult& result) {
  cout << "\n[校验] 外参结果合理性检查\n";
  cout << "════════════════════════════════════════════════════════════\n";

  // 检查旋转矩阵是否为单位正交矩阵
  Mat RTR = result.rotationMatrix.t() * result.rotationMatrix;
  Mat identity = Mat::eye(3, 3, CV_64F);
  double diff = norm(RTR - identity);

  cout << "  [1] 旋转矩阵正交性检查:\n";
  cout << "      计算 R^T*R，应接近单位矩阵\n";
  if (diff < 0.01) {
    cout << "      ✓ PASS (偏差: " << fixed << setprecision(6) << diff << ")\n";
  } else {
    cout << "      ⚠️  WARN (偏差: " << diff << " > 0.01)\n";
  }

  // 检查行列式（应为 +1）
  double det = determinant(result.rotationMatrix);
  cout << "  [2] 旋转矩阵行列式检查:\n";
  cout << "      det(R) 应为 +1 (右手系)\n";
  if (abs(det - 1.0) < 0.01) {
    cout << "      ✓ PASS (det = " << fixed << setprecision(6) << det << ")\n";
  } else {
    cout << "      ⚠️  WARN (det = " << det << " ≠ 1)\n";
  }

  // 检查重投影误差
  cout << "  [3] 重投影误差检查:\n";
  cout << "      平均像素误差 (RMS): " << fixed << setprecision(4)
       << result.reprojectionError << " px\n";
  if (result.reprojectionError < 0.5) {
    cout << "      ✓ PASS - 优秀 (< 0.5 px)\n";
  } else if (result.reprojectionError < 1.0) {
    cout << "      ✓ PASS - 良好 (< 1.0 px)\n";
  } else if (result.reprojectionError < 2.0) {
    cout << "      ⚠️  WARN - 可接受 (< 2.0 px)\n";
  } else {
    cout << "      ❌ FAIL - 需要改进 (≥ 2.0 px)\n";
  }

  cout << "════════════════════════════════════════════════════════════\n\n";
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
  // 解析命令行参数
  if (argc < 3) {
    printUsage(argv[0]);
    return -1;
  }

  string imageFile = argv[1];
  string intrinsicFile = argv[2];
  string worldPointsFile = "";
  string outputFile = "extrinsic.yaml";
  bool autoDetect = true;  // 默认自动检测
  bool manualMark = false;
  bool visualize = false;

  // 解析可选参数
  for (int i = 3; i < argc; ++i) {
    string arg = argv[i];

    if (arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--world-points" && i + 1 < argc) {
      worldPointsFile = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      outputFile = argv[++i];
    } else if (arg == "--auto-detect") {
      autoDetect = true;
      manualMark = false;
    } else if (arg == "--manual-mark") {
      autoDetect = false;
      manualMark = true;
    } else if (arg == "--visualize") {
      visualize = true;
    }
  }

  cout << "\n╔════════════════════════════════════════════════════════════╗\n";
  cout << "║      相机外参标定程序 (自动黑白角点检测) v2.0              ║\n";
  cout << "╚════════════════════════════════════════════════════════════╝\n\n";

  // ====================================================================
  // 步骤 1: 读取相机内参
  // ====================================================================
  cout << "[步骤 1] 读取相机内参...\n";
  cout << "════════════════════════════════════════════════════════════\n";
  CameraIntrinsics intrinsics = readIntrinsicsFromYAML(intrinsicFile);

  if (!intrinsics.isValid) {
    cerr << "[ERROR] 读取内参失败\n";
    return -1;
  }

  cout << "[成功] 相机内参读取完成\n";
  cout << "  相机矩阵:\n";
  cout << intrinsics.cameraMatrix << "\n";
  cout << "  畸变系数:\n";
  cout << intrinsics.distortionCoeffs.t() << "\n\n";

  // ====================================================================
  // 步骤 2: 读取世界坐标系控制点
  // ====================================================================
  cout << "[步骤 2] 读取世界坐标系控制点...\n";
  cout << "════════════════════════════════════════════════════════════\n";

  vector<Point3f> objectPoints;

  if (worldPointsFile.empty()) {
    cout << "[信息] 未提供世界坐标点文件，使用默认 17 个点\n";
    objectPoints = generateDefaultWorldPoints();
    cout << "[信息] 生成的世界坐标点:\n";
    cout << "  点数量: 17 个\n";
    cout << "  坐标系: 右(X) 前(Y) 上(Z)\n";
  } else {
    cout << "[信息] 从文件读取世界坐标点: " << worldPointsFile << "\n";
    objectPoints = readWorldPoints(worldPointsFile);
    if (objectPoints.empty()) {
      cerr << "[ERROR] 读取世界坐标点失败\n";
      return -1;
    }
  }

  cout << "[成功] 世界坐标点数: " << objectPoints.size() << " 个\n";
  cout << "  前 5 个点:\n";
  for (int i = 0; i < min(5, (int)objectPoints.size()); ++i) {
    cout << "    点 " << i << ": (" << fixed << setprecision(2)
         << objectPoints[i].x << ", " << objectPoints[i].y << ", "
         << objectPoints[i].z << ") mm\n";
  }
  cout << "\n";

  // ====================================================================
  // 步骤 3: 读取输入图像
  // ====================================================================
  cout << "[步骤 3] 读取输入图像...\n";
  cout << "════════════════════════════════════════════════════════════\n";
  Mat image = imread(imageFile);

  if (image.empty()) {
    cerr << "[ERROR] 无法读取图像: " << imageFile << endl;
    return -1;
  }

  cout << "[成功] 读取图像: " << imageFile << "\n";
  cout << "  图像尺寸: " << image.cols << " x " << image.rows << " 像素\n";
  cout << "  通道数: " << image.channels() << "\n\n";

  // ====================================================================
  // 步骤 4: 检测或标记图像坐标系控制点
  // ====================================================================
  cout << "[步骤 4] 检测或标记图像坐标系控制点...\n";
  cout << "════════════════════════════════════════════════════════════\n";

  vector<Point2f> imagePoints;

  if (manualMark) {
    cout << "[信息] 使用交互式手动标记模式\n";
    PointMarker marker(image, objectPoints.size());
    marker.run();
    imagePoints = marker.points;

    if (imagePoints.size() != objectPoints.size()) {
      cerr << "[ERROR] 标记点数 (" << imagePoints.size() << ") 与世界坐标点数 ("
           << objectPoints.size() << ") 不匹配\n";
      return -1;
    }
  }

  cout << "[成功] 图像坐标点数: " << imagePoints.size() << " 个\n";
  cout << "  前 5 个点:\n";
  for (int i = 0; i < min(5, (int)imagePoints.size()); ++i) {
    cout << "    点 " << i << ": (" << fixed << setprecision(2)
         << imagePoints[i].x << ", " << imagePoints[i].y << ") px\n";
  }
  cout << "\n";

  // ====================================================================
  // 步骤 5: 计算外参
  // ====================================================================
  ExtrinsicResult result =
      solveExtrinsic(objectPoints, imagePoints, intrinsics.cameraMatrix,
                     intrinsics.distortionCoeffs);

  if (!result.isValid) {
    cerr << "[ERROR] 外参计算失败\n";
    return -1;
  }

  // ====================================================================
  // 步骤 6: 显示结果
  // ====================================================================
  printExtrinsicResult(result);
  validateExtrinsicResult(result);

  // ====================================================================
  // 步骤 7: 保存结果
  // ====================================================================
  saveExtrinsicToYAML(outputFile, result);

  cout << "[完成] 外参标定程序执行完毕\n";
  cout << "════════════════════════════════════════════════════════════\n\n";

  return 0;
}
