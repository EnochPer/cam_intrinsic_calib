/**
 * @file apriltag_batch_process.cpp
 * @brief 批量处理文件夹内图像的 AprilTag 识别和位姿求解程序
 *
 * @details
 *   该程序实现了以下功能：
 *   1. 读取相机内参文件（YAML格式）
 *   2. 读取相机到世界坐标系的外参文件（YAML格式）
 *   3. 顺序读取指定文件夹内所有 .bmp 或 .jpg 图像
 *   4. 使用 apriltag3 库识别图像中的 AprilTag（standart41h12，尺寸0.1m）
 *   5. 计算 AprilTag 在世界坐标系中的位置和姿态
 *   6. 保存结果为 txt 文件
 *   7. 已知 AprilTag 在世界坐标系中的 z 高度不变，添加此约束进行优化
 *
 * 坐标系说明：
 *   - 世界坐标系：右(X) 前(Y) 上(Z)
 *   - 相机坐标系：标准相机坐标系 (X-Right, Y-Down, Z-Forward)
 *   - AprilTag 坐标系：标签中心，四个角为基准
 *
 * @usage
 *   ./apriltag_batch_process <image_folder> <intrinsic_yaml> <extrinsic_yaml>
 * <tag_size> [z_constraint]
 *
 * @example
   ./apriltag_batch_process images/ camera_calib.yaml extrinsic.yaml 0.1
   ./apriltag_batch_process /home/zzh/hikon_cam/picture_data/20251121_174435/ \
 /home/zzh/hikon_cam/camera_calib.yaml /home/zzh/hikon_cam/extrinsic1.yaml \
 0.1 0.215
 */

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// AprilTag 库头文件
extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tagStandard41h12.h>
}

namespace fs = std::filesystem;
using namespace cv;
using namespace std;

// 数据结构（与 apriltag_detector.cpp 保持一致）
struct CameraIntrinsics {
  Mat cameraMatrix;
  Mat distortionCoeffs;
  int imageWidth;
  int imageHeight;
  bool isValid;

  CameraIntrinsics() : isValid(false), imageWidth(0), imageHeight(0) {
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distortionCoeffs = Mat::zeros(5, 1, CV_64F);
  }
};

struct CameraExtrinsics {
  Mat rotationMatrix;
  Mat rotationVector;
  Mat translationVector;
  bool isValid;

  CameraExtrinsics() : isValid(false) {
    rotationMatrix = Mat::eye(3, 3, CV_64F);
    rotationVector = Mat::zeros(3, 1, CV_64F);
    translationVector = Mat::zeros(3, 1, CV_64F);
  }
};

struct AprilTagDetection {
  int tagId;
  vector<Point2f> corners;
  Point2f center;
  Mat rotationMatrix;
  Mat rotationVector;
  Mat translationVector;
  bool isValid;

  AprilTagDetection() : tagId(-1), isValid(false) {
    rotationMatrix = Mat::eye(3, 3, CV_64F);
    rotationVector = Mat::zeros(3, 1, CV_64F);
    translationVector = Mat::zeros(3, 1, CV_64F);
  }
};

struct AprilTagWorldPose {
  int tagId;
  Mat rotationMatrix;
  Mat rotationVector;
  Mat translationVector;
  bool isValid;

  AprilTagWorldPose() : tagId(-1), isValid(false) {
    rotationMatrix = Mat::eye(3, 3, CV_64F);
    rotationVector = Mat::zeros(3, 1, CV_64F);
    translationVector = Mat::zeros(3, 1, CV_64F);
  }
};

// 辅助函数（与 apriltag_detector.cpp 保持一致或修改）

/**
 * @brief 打印程序用法
 */
void printUsage(const char* programName) {
  cout << "\n════════════════════════════════════════════════════════════\n";
  cout << "      AprilTag 批量识别和位姿求解程序 (基于 apriltag3 库)\n";
  cout << "════════════════════════════════════════════════════════════\n\n";
  cout << "用法:\n";
  cout << "  " << programName
       << " <image_folder> <intrinsic_yaml> <extrinsic_yaml>\n";
  cout << "     <tag_size> [z_constraint]\n\n";
  cout << "必需参数:\n";
  cout << "  <image_folder>         输入图像文件夹路径\n";
  cout << "  <intrinsic_yaml>       相机内参文件（YAML格式）\n";
  cout << "  <extrinsic_yaml>       相机外参文件（YAML格式）\n";
  cout << "  <tag_size>             AprilTag 标签尺寸（米，如 0.1）\n\n";
  cout << "可选参数:\n";
  cout << "  <z_constraint>         AprilTag 在世界坐标系中的固定 Z "
          "高度（米，如 1.0）\n";
  cout << "  --help                 显示此帮助信息\n\n";
  cout << "════════════════════════════════════════════════════════════\n\n";
}

/**
 * @brief 从 YAML 文件读取相机内参
 */
CameraIntrinsics readIntrinsicsFromYAML(const string& filename) {
  CameraIntrinsics intrinsics;

  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened()) {
    cerr << "[ERROR] 无法打开内参文件: " << filename << endl;
    return intrinsics;
  }

  try {
    fs["camera_matrix"] >> intrinsics.cameraMatrix;

    if (fs["distortion_coefficients"].empty()) {
      fs["distortion_coeffs"] >> intrinsics.distortionCoeffs;
    } else {
      fs["distortion_coefficients"] >> intrinsics.distortionCoeffs;
    }

    intrinsics.isValid = true;

  } catch (const exception& e) {
    cerr << "[ERROR] 读取内参 YAML 文件异常: " << e.what() << endl;
    intrinsics.isValid = false;
  }

  fs.release();
  return intrinsics;
}

/**
 * @brief 从 YAML 文件读取相机外参
 */
CameraExtrinsics readExtrinsicsFromYAML(const string& filename) {
  CameraExtrinsics extrinsics;

  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened()) {
    cerr << "[ERROR] 无法打开外参文件: " << filename << endl;
    return extrinsics;
  }

  try {
    fs["rotation_matrix"] >> extrinsics.rotationMatrix;
    fs["rotation_vector"] >> extrinsics.rotationVector;
    fs["translation_vector"] >> extrinsics.translationVector;

    extrinsics.isValid = true;

  } catch (const exception& e) {
    cerr << "[ERROR] 读取外参 YAML 文件异常: " << e.what() << endl;
    extrinsics.isValid = false;
  }

  fs.release();
  return extrinsics;
}

/**
 * @brief 检测图像中的 AprilTag
 */
vector<AprilTagDetection> detectAprilTags(const Mat& image, double tagSize,
                                          const Mat& cameraMatrix,
                                          const Mat& distCoeffs,
                                          cv::Point2f bias = {0, 0}) {
  vector<AprilTagDetection> detections;

  // 创建检测器
  apriltag_family_t* tf = tagStandard41h12_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  // 转换为灰度图像
  Mat gray;
  if (image.channels() == 3) {
    cvtColor(image, gray, COLOR_BGR2GRAY);
  } else {
    gray = image.clone();
  }

  // 转换为 apriltag 需要的 image_u8_t 格式
  image_u8_t* img_u8 = image_u8_create(gray.cols, gray.rows);
  for (int y = 0; y < gray.rows; ++y) {
    memcpy(&img_u8->buf[y * img_u8->stride], gray.ptr(y), gray.cols);
  }

  // 检测标记
  zarray_t* detections_array = apriltag_detector_detect(td, img_u8);
  int num_detections = zarray_size(detections_array);

  // 用于位姿估计的相机内参
  apriltag_detection_info_t info;
  info.det = nullptr;
  info.tagsize = tagSize;
  info.fx = cameraMatrix.at<double>(0, 0);
  info.fy = cameraMatrix.at<double>(1, 1);
  info.cx = cameraMatrix.at<double>(0, 2) - bias.x;
  info.cy = cameraMatrix.at<double>(1, 2) - bias.y;

  // 处理每个检测到的标记
  for (int i = 0; i < num_detections; ++i) {
    apriltag_detection_t* det = nullptr;
    zarray_get(detections_array, i, &det);

    if (det == nullptr) continue;

    AprilTagDetection detection;
    detection.tagId = det->id;

    // 提取四个角点和中心
    for (int j = 0; j < 4; ++j) {
      detection.corners.push_back(Point2f(det->p[j][0], det->p[j][1]));
    }
    detection.center = Point2f(det->c[0], det->c[1]);

    // 估计位姿
    info.det = det;
    apriltag_pose_t pose;
    estimate_tag_pose(&info, &pose);

    // 转换位姿到 OpenCV 格式
    detection.rotationMatrix = Mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        detection.rotationMatrix.at<double>(r, c) = MATD_EL(pose.R, r, c);
      }
    }

    detection.translationVector = Mat(3, 1, CV_64F);
    for (int r = 0; r < 3; ++r) {
      detection.translationVector.at<double>(r, 0) = MATD_EL(pose.t, r, 0);
    }

    // 转换为旋转向量
    Rodrigues(detection.rotationMatrix, detection.rotationVector);

    detection.isValid = true;
    detections.push_back(detection);

    // 清理位姿结构体
    matd_destroy(pose.R);
    matd_destroy(pose.t);
  }

  // 清理资源
  image_u8_destroy(img_u8);
  apriltag_detections_destroy(detections_array);
  apriltag_detector_destroy(td);
  tagStandard41h12_destroy(tf);

  return detections;
}

/**
 * @brief 将相机坐标系的位姿变换到世界坐标系
 */
AprilTagWorldPose transformToWorldCoordinates(
    const AprilTagDetection& cameraDetection,
    const CameraExtrinsics& cameraExtrinsics) {
  AprilTagWorldPose worldPose;
  worldPose.tagId = cameraDetection.tagId;

  // 获取相机到世界的变换
  Mat R_w2c = cameraExtrinsics.rotationMatrix;     // 世界→相机 旋转
  Mat t_w2c = cameraExtrinsics.translationVector;  // 世界→相机 平移 (mm)

  Mat R_c2w = R_w2c.t();       // 相机→世界 旋转
  Mat t_c2w = -R_c2w * t_w2c;  // 相机→世界 平移 (mm)

  // 标签在相机坐标系中的位姿
  Mat R_cam2tag = cameraDetection.rotationMatrix;  // 相机→标签 旋转
  Mat t_cam2tag =
      cameraDetection.translationVector.clone() * 1000;  // 转换为 mm

  // 标签在世界坐标系中的旋转和平移
  worldPose.rotationMatrix = R_c2w * R_cam2tag;
  worldPose.translationVector = R_c2w * t_cam2tag + t_c2w;

  // 转换为旋转向量
  Rodrigues(worldPose.rotationMatrix, worldPose.rotationVector);

  worldPose.isValid = true;

  return worldPose;
}

/**
 * @brief 应用 Z 高度约束优化位姿
 */
void applyZConstraint(AprilTagWorldPose& worldPose, double zConstraintMM) {
  // 将 Z 坐标固定为给定值 (mm)
  worldPose.translationVector.at<double>(2, 0) = zConstraintMM;
  // 这里可以添加更复杂的优化，比如重新计算姿态保持 Z 不变
  // 目前简单实现为直接固定 Z 值
}

// 辅助函数：将旋转矩阵转换为四元数
Vec4d rotationMatrixToQuaternion(const Mat& rotationMatrix) {
  double w, x, y, z;

  double trace = rotationMatrix.at<double>(0, 0) +
                 rotationMatrix.at<double>(1, 1) +
                 rotationMatrix.at<double>(2, 2);

  if (trace > 0.0) {
    double s = 0.5 / sqrt(trace + 1.0);
    w = 0.25 / s;
    x = (rotationMatrix.at<double>(2, 1) - rotationMatrix.at<double>(1, 2)) * s;
    y = (rotationMatrix.at<double>(0, 2) - rotationMatrix.at<double>(2, 0)) * s;
    z = (rotationMatrix.at<double>(1, 0) - rotationMatrix.at<double>(0, 1)) * s;
  } else {
    if (rotationMatrix.at<double>(0, 0) > rotationMatrix.at<double>(1, 1) &&
        rotationMatrix.at<double>(0, 0) > rotationMatrix.at<double>(2, 2)) {
      double s = 2.0 * sqrt(1.0 + rotationMatrix.at<double>(0, 0) -
                            rotationMatrix.at<double>(1, 1) -
                            rotationMatrix.at<double>(2, 2));
      w = (rotationMatrix.at<double>(2, 1) - rotationMatrix.at<double>(1, 2)) /
          s;
      x = 0.25 * s;
      y = (rotationMatrix.at<double>(0, 1) + rotationMatrix.at<double>(1, 0)) /
          s;
      z = (rotationMatrix.at<double>(0, 2) + rotationMatrix.at<double>(2, 0)) /
          s;
    } else if (rotationMatrix.at<double>(1, 1) >
               rotationMatrix.at<double>(2, 2)) {
      double s = 2.0 * sqrt(1.0 + rotationMatrix.at<double>(1, 1) -
                            rotationMatrix.at<double>(0, 0) -
                            rotationMatrix.at<double>(2, 2));
      w = (rotationMatrix.at<double>(0, 2) - rotationMatrix.at<double>(2, 0)) /
          s;
      x = (rotationMatrix.at<double>(0, 1) + rotationMatrix.at<double>(1, 0)) /
          s;
      y = 0.25 * s;
      z = (rotationMatrix.at<double>(1, 2) + rotationMatrix.at<double>(2, 1)) /
          s;
    } else {
      double s = 2.0 * sqrt(1.0 + rotationMatrix.at<double>(2, 2) -
                            rotationMatrix.at<double>(0, 0) -
                            rotationMatrix.at<double>(1, 1));
      w = (rotationMatrix.at<double>(1, 0) - rotationMatrix.at<double>(0, 1)) /
          s;
      x = (rotationMatrix.at<double>(0, 2) + rotationMatrix.at<double>(2, 0)) /
          s;
      y = (rotationMatrix.at<double>(1, 2) + rotationMatrix.at<double>(2, 1)) /
          s;
      z = 0.25 * s;
    }
  }

  return Vec4d(x, y, z, w);  // qx qy qz qw
}

/**
 * @brief 保存结果为 TUM 格式文件
 */
void saveResultsToTXT(
    const string& filename,
    const vector<pair<string, vector<AprilTagWorldPose>>>& allResults) {
  ofstream outFile(filename);
  if (!outFile.is_open()) {
    cerr << "[ERROR] 无法打开文件进行写入: " << filename << endl;
    return;
  }

  // TUM 格式：时间戳 tx ty tz qx qy qz qw
  // 时间戳使用图像索引作为伪时间戳，从1开始
  int timestamp = 1;

  for (const auto& result : allResults) {
    const vector<AprilTagWorldPose>& worldPoses = result.second;

    for (const auto& pose : worldPoses) {
      if (pose.isValid) {
        Vec4d quat = rotationMatrixToQuaternion(pose.rotationMatrix);

        outFile << fixed << setprecision(6);

        // 伪时间戳 (s)
        outFile << timestamp << ".0 ";

        // 平移量：转换为米
        outFile << (pose.translationVector.at<double>(0, 0) / 1000.0)
                << " ";  // X (m)
        outFile << (pose.translationVector.at<double>(1, 0) / 1000.0)
                << " ";  // Y (m)
        outFile << (pose.translationVector.at<double>(2, 0) / 1000.0)
                << " ";  // Z (m)

        // 四元数
        outFile << quat[0] << " ";   // qx
        outFile << quat[1] << " ";   // qy
        outFile << quat[2] << " ";   // qz
        outFile << quat[3] << endl;  // qw
      }
    }

    timestamp++;  // 每个图像递增一个时间戳
  }

  outFile.close();
  cout << "[成功] 结果已保存到: " << filename << endl;
}

// 辅助函数：自然排序比较器
bool naturalCompare(const string& a, const string& b) {
  auto it1 = a.begin();
  auto it2 = b.begin();

  while (it1 != a.end() && it2 != b.end()) {
    if (isdigit(*it1) && isdigit(*it2)) {
      // 提取数字序列
      string num1, num2;
      while (it1 != a.end() && isdigit(*it1)) {
        num1.push_back(*it1);
        ++it1;
      }
      while (it2 != b.end() && isdigit(*it2)) {
        num2.push_back(*it2);
        ++it2;
      }

      // 比较数字
      if (num1.size() != num2.size()) {
        return num1.size() < num2.size();
      }
      if (num1 != num2) {
        return stoll(num1) < stoll(num2);
      }
    } else {
      if (*it1 != *it2) {
        return *it1 < *it2;
      }
      ++it1;
      ++it2;
    }
  }

  return a.size() < b.size();
}

/**
 * @brief 列出文件夹内所有 .bmp 和 .jpg 图像文件
 */
vector<string> listImageFiles(const string& folderPath) {
  vector<string> imageFiles;

  try {
    for (const auto& entry : fs::directory_iterator(folderPath)) {
      if (entry.is_regular_file()) {
        string filename = entry.path().string();
        string extension = entry.path().extension().string();

        // 转换为小写
        transform(extension.begin(), extension.end(), extension.begin(),
                  ::tolower);

        if (extension == ".bmp" || extension == ".jpg" ||
            extension == ".jpeg") {
          imageFiles.push_back(filename);
        }
      }
    }
  } catch (const fs::filesystem_error& e) {
    cerr << "[ERROR] 遍历文件夹失败: " << e.what() << endl;
  }

  // 按自然排序
  sort(imageFiles.begin(), imageFiles.end(), naturalCompare);

  return imageFiles;
}

/**
 * @brief 绘制 AprilTag 在 XOY 世界坐标系的轨迹俯视图（EVO 风格）
 */
void visualizeTrajectory(
    const vector<pair<string, vector<AprilTagWorldPose>>>& allResults,
    const string& outputFile) {
  // 按标签 ID 分类轨迹点
  map<int, vector<Point3d>> tagTrajectories;  // 包含3D坐标点

  // 收集所有轨迹点
  for (const auto& result : allResults) {
    const vector<AprilTagWorldPose>& worldPoses = result.second;

    for (const auto& pose : worldPoses) {
      if (pose.isValid) {
        double x = pose.translationVector.at<double>(0, 0) / 1000.0;  // X (m)
        double y = pose.translationVector.at<double>(1, 0) / 1000.0;  // Y (m)
        double z = pose.translationVector.at<double>(2, 0) / 1000.0;  // Z (m)
        tagTrajectories[pose.tagId].emplace_back(x, y, z);
      }
    }
  }

  if (tagTrajectories.empty()) {
    cerr << "[WARNING] 没有足够的轨迹点进行可视化\n";
    return;
  }

  // 设置画布参数
  const int canvasSize = 1000;
  const int margin = 80;
  Mat canvas =
      Mat::zeros(canvasSize + 2 * margin, canvasSize + 2 * margin, CV_8UC3);
  Scalar backgroundColor(245, 245, 245);  // 浅灰色背景（EVO风格）
  canvas.setTo(backgroundColor);

  // 设置固定的坐标范围
  double minX = 0.0;  // X轴范围 0 到 2.5 米
  double maxX = 2.7;
  double minY = 0.0;  // Y轴范围 0 到 3.0 米
  double maxY = 3.1;

  // Z轴范围仍根据数据自动计算
  double minZ = INFINITY, maxZ = -INFINITY;
  for (const auto& [tagId, points] : tagTrajectories) {
    for (const auto& point : points) {
      minZ = min(minZ, point.z);
      maxZ = max(maxZ, point.z);
    }
  }

  if (minZ == INFINITY) minZ = 0.0;
  if (maxZ == -INFINITY) maxZ = 1.0;

  // 计算范围
  double rangeX = maxX - minX;
  double rangeY = maxY - minY;
  double rangeZ = maxZ - minZ;

  // 计算缩放因子
  double scale = min(canvasSize / rangeX, canvasSize / rangeY);

  // 绘制网格线 - 固定间隔0.2米
  Scalar gridColor(200, 200, 200);  // 灰色网格
  double gridInterval = 0.2;        // 固定间隔0.2米

  // X轴方向网格线（水平线）
  double roundedIntervalY = gridInterval;

  for (double y = minY; y <= maxY; y += roundedIntervalY) {
    int canvasY = static_cast<int>(canvasSize - (y - minY) * scale) + margin;
    line(canvas, Point(margin, canvasY), Point(canvasSize + margin, canvasY),
         gridColor, 1);

    // 绘制刻度标签
    string label = to_string(y).substr(0, 5);
    putText(canvas, label, Point(margin - 50, canvasY + 5),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
  }

  // Y轴方向网格线（垂直线）
  double roundedIntervalX = roundedIntervalY;

  for (double x = minX; x <= maxX; x += roundedIntervalX) {
    int canvasX = static_cast<int>((x - minX) * scale) + margin;
    line(canvas, Point(canvasX, margin), Point(canvasX, canvasSize + margin),
         gridColor, 1);

    // 绘制刻度标签
    string label = to_string(x).substr(0, 5);
    putText(canvas, label, Point(canvasX - 20, canvasSize + margin + 25),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
  }

  // 绘制轨迹 - EVO 风格：使用不同风格的线条
  vector<Scalar> colors = {
      Scalar(255, 0, 0),    // 红色 - 主轨迹
      Scalar(0, 0, 255),    // 蓝色 - 参考轨迹
      Scalar(0, 128, 0),    // 深绿色 - 另一条轨迹
      Scalar(128, 0, 128),  // 深紫色
      Scalar(128, 128, 0)   // 深黄色
  };

  for (size_t i = 0; i < tagTrajectories.size(); ++i) {
    auto it = tagTrajectories.begin();
    advance(it, i);
    int tagId = it->first;
    const vector<Point3d>& points = it->second;
    Scalar color = colors[i % colors.size()];

    // 选择线条类型
    int thickness = 2;
    int lineType = LINE_8;

    if (i == 1) {  // 第二条轨迹用AA线条，作为参考
      lineType = LINE_AA;
    }

    // 绘制轨迹线
    if (points.size() > 1) {
      for (size_t j = 1; j < points.size(); ++j) {
        Point3d p1 = points[j - 1];
        Point3d p2 = points[j];

        int x1 = static_cast<int>((p1.x - minX) * scale) + margin;
        int y1 = static_cast<int>(canvasSize - (p1.y - minY) * scale) + margin;

        int x2 = static_cast<int>((p2.x - minX) * scale) + margin;
        int y2 = static_cast<int>(canvasSize - (p2.y - minY) * scale) + margin;

        line(canvas, Point(x1, y1), Point(x2, y2), color, thickness, lineType);
      }
    }
  }

  // 绘制坐标轴（EVO风格，用箭头）
  Scalar axisColor(0, 0, 0);  // 黑色坐标轴
  int axisLength = 60;        // 箭头长度

  // X轴
  Point xStart(margin, canvasSize + margin);
  Point xEnd(margin + axisLength, canvasSize + margin);
  arrowedLine(canvas, xStart, xEnd, axisColor, 2, LINE_AA, 0, 0.1);
  putText(canvas, "x (m)", Point(xEnd.x + 5, xEnd.y + 15), FONT_HERSHEY_SIMPLEX,
          0.6, axisColor, 2);

  // Y轴
  Point yStart(margin, canvasSize + margin);
  Point yEnd(margin, canvasSize + margin - axisLength);
  arrowedLine(canvas, yStart, yEnd, axisColor, 2, LINE_AA, 0, 0.1);
  putText(canvas, "y (m)", Point(yEnd.x - 30, yEnd.y - 5), FONT_HERSHEY_SIMPLEX,
          0.6, axisColor, 2);

  // 保存图像
  if (!imwrite(outputFile, canvas)) {
    cerr << "[ERROR] 无法保存轨迹图像: " << outputFile << endl;
    return;
  }

  cout << "[成功] 轨迹图像已保存到: " << outputFile << endl;
}

int main(int argc, char* argv[]) {
  if (argc < 5) {
    printUsage(argv[0]);
    return -1;
  }

  string imageFolder = argv[1];
  string intrinsicFile = argv[2];
  string extrinsicFile = argv[3];
  double tagSize = atof(argv[4]);
  double zConstraint = -1.0;  // 默认无约束

  if (argc >= 6) {
    zConstraint = atof(argv[5]);
  }

  // 读取相机内参
  cout << "[步骤 1] 读取相机内参...\n";
  CameraIntrinsics intrinsics = readIntrinsicsFromYAML(intrinsicFile);
  if (!intrinsics.isValid) {
    cerr << "[ERROR] 读取内参失败\n";
    return -1;
  }
  cout << "[成功] 读取相机内参: " << intrinsicFile << "\n\n";

  // 读取相机外参
  cout << "[步骤 2] 读取相机外参...\n";
  CameraExtrinsics extrinsics = readExtrinsicsFromYAML(extrinsicFile);
  if (!extrinsics.isValid) {
    cerr << "[ERROR] 读取外参失败\n";
    return -1;
  }
  cout << "[成功] 读取相机外参: " << extrinsicFile << "\n\n";

  // 列出文件夹内的图像文件
  cout << "[步骤 3] 遍历图像文件夹...\n";
  vector<string> imageFiles = listImageFiles(imageFolder);
  if (imageFiles.empty()) {
    cerr << "[ERROR] 文件夹内没有 .bmp 或 .jpg 图像\n";
    return -1;
  }
  cout << "[成功] 找到 " << imageFiles.size() << " 个图像文件\n\n";

  // 批量处理图像
  cout << "[步骤 4] 批量处理图像...\n";
  vector<pair<string, vector<AprilTagWorldPose>>> allResults;

  for (const string& imageFile : imageFiles) {
    cout << "  处理图像: " << imageFile << endl;

    Mat image = imread(imageFile);
    if (image.empty()) {
      cerr << "  [ERROR] 无法读取图像: " << imageFile << endl;
      continue;
    }

    // // 对图像进行ROI裁切
    // Rect roi(60, 450, 2400, 1600);  // x=60, y=450, width=2400, height=1600
    // Mat croppedImage = image(roi);
    cv::Point2f bias = {60, 450};

    // 检测 AprilTag
    vector<AprilTagDetection> detections =
        detectAprilTags(image, tagSize, intrinsics.cameraMatrix,
                        intrinsics.distortionCoeffs, bias);

    // 保存检测到AprilTag的图像
    if (!detections.empty()) {
      // 创建detect目录
      string detectDir = imageFolder + "/detect";
      fs::create_directory(detectDir);

      // 提取原始文件名并构造保存路径
      string basename = fs::path(imageFile).filename().string();
      string savePath = detectDir + "/" + basename;

      // 绘制AprilTag的四个角点和中心点
      for (const auto& detection : detections) {
        // 绘制四个角点
        for (int i = 0; i < 4; ++i) {
          circle(image, detection.corners[i], 5, Scalar(0, 255, 0),
                 -1);  // 绿色圆点标记角点
        }
        // 绘制中心点
        circle(image, detection.center, 8, Scalar(255, 0, 0),
               -1);  // 蓝色圆点标记中心点
      }

      // 保存绘制后的图像
      imwrite(savePath, image);
      cout << "  [成功] 检测图像已保存到: " << savePath << endl;
    }

    if (detections.empty()) {
      cout << "  [信息] 未检测到 AprilTag\n";
      continue;
    }

    // 变换到世界坐标系
    vector<AprilTagWorldPose> worldPoses;
    for (const auto& detection : detections) {
      AprilTagWorldPose worldPose =
          transformToWorldCoordinates(detection, extrinsics);

      // 应用 Z 高度约束
      if (zConstraint > 0.0) {
        double zConstraintMM = zConstraint * 1000;  // 转换为 mm
        applyZConstraint(worldPose, zConstraintMM);
      }

      worldPoses.push_back(worldPose);
    }

    allResults.emplace_back(imageFile, worldPoses);
  }

  // 保存结果为 TXT 文件
  cout << "\n[步骤 5] 保存结果...\n";
  string outputFile = "apriltag_batch_results.txt";
  saveResultsToTXT(outputFile, allResults);

  // 可视化轨迹
  cout << "[步骤 6] 可视化轨迹...\n";
  string trajectoryImageFile = "apriltag_trajectory_topview.png";
  visualizeTrajectory(allResults, trajectoryImageFile);

  cout << "\n[完成] 批量处理完毕\n";
  return 0;
}
