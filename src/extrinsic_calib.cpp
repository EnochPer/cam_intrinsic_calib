/**
 * @file extrinsic_calib.cpp
 * @brief 相机外参标定程序 - 已知世界坐标系控制点，求相机姿态
 *
 * @details
 *   该程序实现了以下功能：
 *   1. 读取相机内参文件（YAML格式）
 *   2. 读取世界坐标系下的控制点坐标 (20个已测量的1×1棋盘格点)
 *   3. 从图像中手动标记或自动检测这些点在图像中的像素坐标
 *   4. 使用 solvePnP 计算相机的外参（旋转矩阵 R 和平移向量 t）
 *   5. 保存外参结果到 YAML 文件
 *
 * 世界坐标系：
 *   - 棋盘格起点在原点 (0, 0, 0)
 *   - X轴正方向：棋盘格向右
 *   - Y轴正方向：棋盘格向上
 *   - Z轴正方向：向外（右手系）
 *   - 每个方格大小：1mm × 1mm
 *
 * @usage
 *   ./extrinsic_calib <image_file> <intrinsic_yaml> [options]
 *   ./extrinsic_calib <image_file> <intrinsic_yaml> --world-points
 * world_points.txt
 *   ./extrinsic_calib <image_file> <intrinsic_yaml> --image-points
 * image_points.txt
 *   ./extrinsic_calib <image_file> <intrinsic_yaml> --output extrinsic.yaml
 *
 * @example
 *   ./extrinsic_calib photo.jpg camera_calibration.yaml --world-points
 * points_3d.txt
 *   ./extrinsic_calib photo.jpg camera_calibration.yaml --world-points
 * points_3d.txt --image-points points_2d.txt --output extrinsic.yaml
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

using namespace cv;
using namespace std;

// ============================================================================
// 数据结构
// ============================================================================

/// 相机内参结构体
struct CameraIntrinsics {
  Mat cameraMatrix;      ///< 3×3 相机矩阵
  Mat distortionCoeffs;  ///< 畸变系数 (k1, k2, p1, p2, k3, ...)
  int imageWidth;        ///< 图像宽度
  int imageHeight;       ///< 图像高度
  bool isValid;          ///< 是否有效
  string description;    ///< 描述信息

  CameraIntrinsics() : isValid(false), imageWidth(0), imageHeight(0) {
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distortionCoeffs = Mat::zeros(5, 1, CV_64F);
  }
};

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
  cout << "\n========================================\n";
  cout << "相机外参标定程序\n";
  cout << "========================================\n\n";
  cout << "用法:\n";
  cout << "  " << programName << " <image_file> <intrinsic_yaml> [options]\n\n";
  cout << "必需参数:\n";
  cout << "  <image_file>           输入图像文件（包含棋盘格）\n";
  cout << "  <intrinsic_yaml>       相机内参文件（YAML格式）\n\n";
  cout << "可选参数:\n";
  cout << "  --world-points <file>  世界坐标系控制点文件\n";
  cout
      << "                         格式：每行一个点，3个值用空格分隔 (x y z)\n";
  cout << "                         默认：自动生成 4×5 的 1×1 棋盘格点 "
          "(20个点)\n";
  cout << "  --image-points <file>  图像坐标系控制点文件\n";
  cout << "                         格式：每行一个点，2个值用空格分隔 (x y)\n";
  cout << "                         如果不提供，会要求用户在图像上点击标记\n";
  cout << "  --output <file>        输出外参文件名 (默认: extrinsic.yaml)\n";
  cout << "  --show-board           显示棋盘格点标记结果\n";
  cout << "  --help                 显示此帮助信息\n\n";
  cout << "文件格式示例:\n";
  cout << "  world_points.txt (世界坐标，单位mm):\n";
  cout << "    0 0 0\n";
  cout << "    1 0 0\n";
  cout << "    2 0 0\n";
  cout << "    ...\n\n";
  cout << "  image_points.txt (图像坐标，单位像素):\n";
  cout << "    100.5 200.3\n";
  cout << "    150.2 200.1\n";
  cout << "    200.1 200.4\n";
  cout << "    ...\n\n";
  cout << "========================================\n\n";
}

/**
 * @brief 从 YAML 文件读取相机内参
 * @param filename YAML 文件路径
 * @return 相机内参结构体
 */
CameraIntrinsics readIntrinsicsFromYAML(const string& filename) {
  CameraIntrinsics intrinsics;

  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened()) {
    cerr << "[ERROR] 无法打开内参文件: " << filename << endl;
    return intrinsics;
  }

  try {
    // 读取相机矩阵
    fs["camera_matrix"] >> intrinsics.cameraMatrix;

    // 读取畸变系数
    if (fs["distortion_coefficients"].empty()) {
      fs["distortion_coeffs"] >> intrinsics.distortionCoeffs;
    } else {
      fs["distortion_coefficients"] >> intrinsics.distortionCoeffs;
    }

    // 读取图像尺寸
    if (!fs["image_width"].empty()) {
      intrinsics.imageWidth = (int)fs["image_width"];
    }
    if (!fs["image_height"].empty()) {
      intrinsics.imageHeight = (int)fs["image_height"];
    }

    intrinsics.isValid = true;
    intrinsics.description = "[成功] 读取相机内参: " + filename;

  } catch (const exception& e) {
    cerr << "[ERROR] 读取 YAML 文件异常: " << e.what() << endl;
    intrinsics.isValid = false;
  }

  fs.release();
  return intrinsics;
}

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

/**
 * @brief 从文本文件读取图像坐标系控制点
 * @param filename 文本文件路径
 * @return 2D 点列表
 */
vector<Point2f> readImagePoints(const string& filename) {
  vector<Point2f> points;
  ifstream file(filename);

  if (!file.is_open()) {
    cerr << "[ERROR] 无法打开图像坐标点文件: " << filename << endl;
    return points;
  }

  float x, y;
  while (file >> x >> y) {
    points.push_back(Point2f(x, y));
  }

  file.close();

  cout << "[INFO] 读取图像坐标点: " << points.size() << " 个\n";
  return points;
}

/**
 * @brief 生成默认的世界坐标系控制点（4×5 的 1×1 棋盘格）
 * @return 3D 点列表 (20 个点)
 *
 * 棋盘格排列：
 *   (0,4)  (1,4)  (2,4)  (3,4)  (4,4)
 *   (0,3)  (1,3)  (2,3)  (3,3)  (4,3)
 *   (0,2)  (1,2)  (2,2)  (3,2)  (4,2)
 *   (0,1)  (1,1)  (2,1)  (3,1)  (4,1)
 *   (0,0)  (1,0)  (2,0)  (3,0)  (4,0)
 */
vector<Point3f> generateDefaultWorldPoints() {
  vector<Point3f> points;
  // 4 行 5 列的 1×1 棋盘格
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 5; ++col) {
      points.push_back(Point3f(col, row, 0.0f));
    }
  }
  return points;
}

/**
 * @brief 在图像上交互式标记控制点
 * @param image 输入图像
 * @param points 输出的图像坐标点列表
 * @param worldPointCount 需要标记的点数
 *
 * 鼠标点击图像上的控制点位置进行标记
 */
class PointMarker {
 public:
  vector<Point2f> points;
  Mat image;
  Mat originalImage;  // 保存原始图像，用于撤销时重绘
  int pointCount;
  int markedCount;

  PointMarker(const Mat& img, int count)
      : image(img.clone()),
        originalImage(img.clone()),
        pointCount(count),
        markedCount(0) {}

  static void mouseCallback(int event, int x, int y, int flags,
                            void* userdata) {
    PointMarker* marker = static_cast<PointMarker*>(userdata);

    if (event == EVENT_LBUTTONDOWN &&
        marker->markedCount < marker->pointCount) {
      marker->points.push_back(Point2f(x, y));
      marker->markedCount++;

      // 在图像上绘制标记
      circle(marker->image, Point(x, y), 5, Scalar(0, 255, 0), -1);
      putText(marker->image, to_string(marker->markedCount),
              Point(x + 10, y - 10), FONT_HERSHEY_SIMPLEX, 0.5,
              Scalar(0, 255, 0), 2);

      cout << "[标记 " << marker->markedCount << "/" << marker->pointCount
           << "] 点坐标: (" << x << ", " << y << ")\n";

      imshow("标记控制点 (click to mark)", marker->image);

      if (marker->markedCount == marker->pointCount) {
        cout << "[完成] 所有 " << marker->pointCount << " 个点已标记\n";
      }
    } else if (event == EVENT_RBUTTONDOWN) {
      // 右键撤销最后一个点
      if (marker->markedCount > 0) {
        marker->points.pop_back();
        marker->markedCount--;

        // 重新绘制（使用原始图像）
        marker->image = marker->originalImage.clone();
        for (int i = 0; i < marker->markedCount; ++i) {
          circle(marker->image, marker->points[i], 5, Scalar(0, 255, 0), -1);
          putText(marker->image, to_string(i + 1),
                  Point(marker->points[i].x + 10, marker->points[i].y - 10),
                  FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
        }
        cout << "[撤销] 当前已标记 " << marker->markedCount << " 个点\n";
        imshow("标记控制点 (click to mark)", marker->image);
      }
    }
  }

  void run() {
    namedWindow("标记控制点 (click to mark)");
    setMouseCallback("标记控制点 (click to mark)", mouseCallback, this);

    cout << "\n[操作说明]\n";
    cout << "  - 左键点击：标记控制点\n";
    cout << "  - 右键点击：撤销最后一个点\n";
    cout << "  - 按 'q' 或 'ESC' 完成标记\n";
    cout << "  - 需要标记 " << pointCount << " 个点\n\n";

    imshow("标记控制点 (click to mark)", image);

    while (markedCount < pointCount) {
      int key = waitKey(0);
      if (key == 'q' || key == 27) {  // 'q' 或 ESC
        if (markedCount == pointCount) {
          break;
        } else {
          cout << "[提示] 还需要标记 " << (pointCount - markedCount)
               << " 个点\n";
        }
      }
    }

    destroyWindow("标记控制点 (click to mark)");
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
  cout << "========================================\n";
  cout << "  控制点数: " << objectPoints.size() << "\n";
  cout << "  求解方法: SOLVEPNP_ITERATIVE (迭代法，精度高)\n";

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
  cout << "========================================\n\n";

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
  cout << "\n========================================\n";
  cout << "外参标定结果\n";
  cout << "========================================\n\n";

  cout << "────── 旋转矩阵 (Rotation Matrix) ──────\n";
  cout << "单位: 无量纲\n";
  cout << "形式: 3×3 矩阵\n";
  cout << result.rotationMatrix << "\n\n";

  cout << "────── 旋转向量 (Rotation Vector) ──────\n";
  cout << "单位: 弧度\n";
  cout << "形式: 3×1 向量 (Rodrigues 表示)\n";
  cout << "说明: 向量方向为旋转轴，向量长度为旋转角度\n";
  cout << result.rotationVector << "\n\n";

  // 从旋转向量计算欧拉角（仅供参考）
  Mat rotationMatrix = result.rotationMatrix;
  double angle = norm(result.rotationVector);
  double angleDeg = angle * 180.0 / M_PI;
  cout << "  旋转角度: " << fixed << setprecision(4) << angleDeg << " 度 ("
       << angle << " 弧度)\n\n";

  cout << "────── 平移向量 (Translation Vector) ──────\n";
  cout << "单位: 毫米\n";
  cout << "形式: 3×1 向量\n";
  cout << "说明: 世界坐标系原点在相机坐标系中的位置\n";
  cout << "  tx = " << result.translationVector.at<double>(0, 0) << " mm\n";
  cout << "  ty = " << result.translationVector.at<double>(1, 0) << " mm\n";
  cout << "  tz = " << result.translationVector.at<double>(2, 0) << " mm\n\n";

  cout << "────── 标定精度 ──────\n";
  cout << "重投影误差 (RMS): " << fixed << setprecision(4)
       << result.reprojectionError << " px\n";
  cout << "说明: 误差越小说明外参精度越高，通常 < 1.0px 为优\n\n";

  cout << "========================================\n\n";
}

/**
 * @brief 验证外参结果
 */
void validateExtrinsicResult(const ExtrinsicResult& result) {
  cout << "[校验] 外参结果合理性检查\n";
  cout << "========================================\n";

  // 检查旋转矩阵是否为单位正交矩阵
  Mat RTR = result.rotationMatrix.t() * result.rotationMatrix;
  Mat identity = Mat::eye(3, 3, CV_64F);
  double diff = norm(RTR - identity);

  cout << "  旋转矩阵正交性: ";
  if (diff < 0.01) {
    cout << "✓ 正常 (偏差: " << diff << ")\n";
  } else {
    cout << "⚠️  警告 (偏差: " << diff << " > 0.01)\n";
  }

  // 检查行列式（应为 +1）
  double det = determinant(result.rotationMatrix);
  cout << "  旋转矩阵行列式: ";
  if (abs(det - 1.0) < 0.01) {
    cout << "✓ 正常 (" << det << ")\n";
  } else {
    cout << "⚠️  警告 (" << det << " ≠ 1)\n";
  }

  // 检查重投影误差
  cout << "  重投影误差: ";
  if (result.reprojectionError < 1.0) {
    cout << "✓ 优秀 (" << result.reprojectionError << " px < 1.0 px)\n";
  } else if (result.reprojectionError < 2.0) {
    cout << "⚠️  良好 (" << result.reprojectionError << " px < 2.0 px)\n";
  } else {
    cout << "❌ 需要改进 (" << result.reprojectionError << " px ≥ 2.0 px)\n";
  }

  cout << "========================================\n\n";
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
  string imagePointsFile = "";
  string outputFile = "extrinsic.yaml";
  bool showBoard = false;

  // 解析可选参数
  for (int i = 3; i < argc; ++i) {
    string arg = argv[i];

    if (arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--world-points" && i + 1 < argc) {
      worldPointsFile = argv[++i];
    } else if (arg == "--image-points" && i + 1 < argc) {
      imagePointsFile = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      outputFile = argv[++i];
    } else if (arg == "--show-board") {
      showBoard = true;
    }
  }

  cout << "\n";
  cout << "╔════════════════════════════════════════╗\n";
  cout << "║       相机外参标定程序 v1.0            ║\n";
  cout << "╚════════════════════════════════════════╝\n\n";

  // ====================================================================
  // 步骤 1: 读取相机内参
  // ====================================================================
  cout << "[步骤 1] 读取相机内参...\n";
  cout << "========================================\n";
  CameraIntrinsics intrinsics = readIntrinsicsFromYAML(intrinsicFile);

  if (!intrinsics.isValid) {
    cerr << "[ERROR] 读取内参失败\n";
    return -1;
  }

  cout << "[成功] " << intrinsics.description << "\n";
  cout << "  相机矩阵:\n";
  cout << intrinsics.cameraMatrix << "\n";
  cout << "  畸变系数:\n";
  cout << intrinsics.distortionCoeffs.t() << "\n\n";

  // ====================================================================
  // 步骤 2: 读取世界坐标系控制点
  // ====================================================================
  cout << "[步骤 2] 读取世界坐标系控制点...\n";
  cout << "========================================\n";

  vector<Point3f> objectPoints;

  if (worldPointsFile.empty()) {
    cout << "[信息] 未提供世界坐标点文件，使用默认 4×5 棋盘格 (20 个点)\n";
    objectPoints = generateDefaultWorldPoints();
    cout << "[信息] 生成的世界坐标点:\n";
    cout << "  棋盘格大小: 5 列 × 4 行\n";
    cout << "  方格尺寸: 1mm × 1mm\n";
    cout << "  点范围: (0,0,0) 到 (4,3,0)\n";
  } else {
    cout << "[信息] 从文件读取世界坐标点: " << worldPointsFile << "\n";
    objectPoints = readWorldPoints(worldPointsFile);
    if (objectPoints.empty()) {
      cerr << "[ERROR] 读取世界坐标点失败\n";
      return -1;
    }
  }

  cout << "[信息] 世界坐标点数: " << objectPoints.size() << "\n";
  cout << "  前 5 个点:\n";
  for (int i = 0; i < min(5, (int)objectPoints.size()); ++i) {
    cout << "    点 " << i << ": (" << objectPoints[i].x << ", "
         << objectPoints[i].y << ", " << objectPoints[i].z << ")\n";
  }
  cout << "\n";

  // ====================================================================
  // 步骤 3: 读取图像
  // ====================================================================
  cout << "[步骤 3] 读取输入图像...\n";
  cout << "========================================\n";
  Mat image = imread(imageFile);

  if (image.empty()) {
    cerr << "[ERROR] 无法读取图像: " << imageFile << endl;
    return -1;
  }

  cout << "[成功] 读取图像: " << imageFile << "\n";
  cout << "  图像尺寸: " << image.cols << " x " << image.rows << "\n";
  cout << "  通道数: " << image.channels() << "\n\n";

  // ====================================================================
  // 步骤 4: 读取或标记图像坐标系控制点
  // ====================================================================
  cout << "[步骤 4] 读取图像坐标系控制点...\n";
  cout << "========================================\n";

  vector<Point2f> imagePoints;

  if (!imagePointsFile.empty()) {
    cout << "[信息] 从文件读取图像坐标点: " << imagePointsFile << "\n";
    imagePoints = readImagePoints(imagePointsFile);
    if (imagePoints.empty()) {
      cerr << "[ERROR] 读取图像坐标点失败\n";
      return -1;
    }
  } else {
    cout << "[信息] 交互式标记图像坐标点\n";
    PointMarker marker(image, objectPoints.size());
    marker.run();
    imagePoints = marker.points;

    if (imagePoints.size() != objectPoints.size()) {
      cerr << "[ERROR] 标记点数 (" << imagePoints.size() << ") 与世界坐标点数 ("
           << objectPoints.size() << ") 不匹配\n";
      return -1;
    }
  }

  cout << "[成功] 图像坐标点数: " << imagePoints.size() << "\n";
  cout << "  前 5 个点:\n";
  for (int i = 0; i < min(5, (int)imagePoints.size()); ++i) {
    cout << "    点 " << i << ": (" << fixed << setprecision(2)
         << imagePoints[i].x << ", " << imagePoints[i].y << ")\n";
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
  cout << "========================================\n\n";

  return 0;
}
