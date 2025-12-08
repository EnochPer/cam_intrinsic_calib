/**
 * @file apriltag_detector.cpp
 * @brief AprilTag 识别和位姿求解程序（使用官方 apriltag3 库）
 *
 * @details
 *   该程序实现了以下功能：
 *   1. 读取相机内参文件（YAML格式）
 *   2. 读取相机的外参文件（相对世界坐标系）
 *   3. 使用官方 apriltag 库识别图像中的 AprilTag 标记
 *   4. 计算 AprilTag 在相机坐标系中的位姿
 *   5. 通过相机外参变换，计算 AprilTag 在世界坐标系中的位姿
 *   6. 显示识别结果和位姿信息
 *
 * 坐标系说明：
 *   - 世界坐标系：右(X) 前(Y) 上(Z)
 *   - 相机坐标系：标准相机坐标系 (X-Right, Y-Down, Z-Forward)
 *   - AprilTag 坐标系：标签中心，四个角为基准
 *
 * @usage
 *   ./apriltag_detector <image_file> <intrinsic_yaml> <extrinsic_yaml>
 *   <tag_size> [options]
 *
 * @example
 *   ./apriltag_detector photo.jpg camera_calib.yaml extrinsic.yaml 0.15
 *   ./apriltag_detector photo.jpg camera_calib.yaml extrinsic.yaml 0.15
 * --output result.yaml
 *   ./apriltag_detector photo.jpg camera_calib.yaml extrinsic.yaml 0.15
 * --tag-family 41h12 --visualize
 */

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// 引入共享的相机参数读取函数
#include "../include/calib_base.h"

// AprilTag 库头文件（从系统安装的 apriltag 项目中获取）
extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
}

using namespace cv;
using namespace std;

// ============================================================================
// 数据结构
// ============================================================================

/// AprilTag 检测结果结构体
struct AprilTagDetection {
  int tagId;                ///< 标签 ID
  vector<Point2f> corners;  ///< 图像中的四个角点
  Point2f center;           ///< 图像中的中心点
  Mat rotationMatrix;       ///< 3×3 旋转矩阵 (相机→标签)
  Mat rotationVector;       ///< 3×1 旋转向量
  Mat translationVector;    ///< 3×1 平移向量 (相机坐标系，单位: m)
  bool isValid;             ///< 是否有效

  AprilTagDetection() : tagId(-1), isValid(false) {
    rotationMatrix = Mat::eye(3, 3, CV_64F);
    rotationVector = Mat::zeros(3, 1, CV_64F);
    translationVector = Mat::zeros(3, 1, CV_64F);
  }
};

/// AprilTag 在世界坐标系中的位姿结构体
struct AprilTagWorldPose {
  int tagId;              ///< 标签 ID
  Mat rotationMatrix;     ///< 3×3 旋转矩阵 (世界→标签)
  Mat rotationVector;     ///< 3×1 旋转向量
  Mat translationVector;  ///< 3×1 平移向量 (世界坐标系，单位: mm)
  bool isValid;           ///< 是否有效

  AprilTagWorldPose() : tagId(-1), isValid(false) {
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
  cout << "      AprilTag 识别和位姿求解程序 (基于 apriltag3 库)\n";
  cout << "════════════════════════════════════════════════════════════\n\n";
  cout << "用法:\n";
  cout << "  " << programName
       << " <image_file> <intrinsic_yaml> <extrinsic_yaml>\n";
  cout << "     <tag_size> [options]\n\n";
  cout << "必需参数:\n";
  cout << "  <image_file>           输入图像文件（包含 AprilTag）\n";
  cout << "  <intrinsic_yaml>       相机内参文件（YAML格式）\n";
  cout << "  <extrinsic_yaml>       相机外参文件（YAML格式）\n";
  cout
      << "  <tag_size>             AprilTag 标签尺寸（米，如 0.15 = 15cm）\n\n";
  cout << "可选参数:\n";
  cout << "  --tag-family <family>  AprilTag 标签族\n";
  cout << "                         支持: 16h5, 25h9, 36h10, 36h11,\n";
  cout << "                                circle21h7, circle49h12,\n";
  cout << "                                custom48h12,\n";
  cout << "                                41h12 (推荐, 默认),\n";
  cout << "                                52h13\n";
  cout << "  --output <file>        输出结果文件 (YAML格式)\n";
  cout << "  --visualize            显示识别结果和坐标系\n";
  cout << "  --help                 显示此帮助信息\n\n";
  cout << "════════════════════════════════════════════════════════════\n\n";
}

/**
 * @brief 检测图像中的 AprilTag
 */
vector<AprilTagDetection> detectAprilTags(const Mat& image, double tagSize,
                                          const Mat& cameraMatrix,
                                          const Mat& distCoeffs,
                                          apriltag_family_t* tf) {
  vector<AprilTagDetection> detections;

  cout << "[步骤] 检测 AprilTag 标记...\n";
  cout << "════════════════════════════════════════════════════════════\n";
  cout << "  标签尺寸: " << fixed << setprecision(4) << tagSize << " m\n";

  // 创建检测器
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

  cout << "  检测到 " << num_detections << " 个标记\n";

  if (num_detections == 0) {
    cout << "  [警告] 未检测到任何 AprilTag\n";
    cout << "════════════════════════════════════════════════════════════\n";
  } else {
    cout << "  [成功] " << num_detections << " 个标记检测完毕\n";
    cout << "════════════════════════════════════════════════════════════\n";

    // 获取相机内参以用于位姿估计
    apriltag_detection_info_t info;
    info.det = nullptr;
    info.tagsize = tagSize;
    info.fx = cameraMatrix.at<double>(0, 0);
    info.fy = cameraMatrix.at<double>(1, 1);
    info.cx = cameraMatrix.at<double>(0, 2);
    info.cy = cameraMatrix.at<double>(1, 2);

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
      // pose.R 是 matd_t（3×3 矩阵），pose.t 是 matd_t（3×1 向量）
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

      cout << "  ├─ 标记 ID: " << detection.tagId << "\n";
      cout << "  │  位置 (相机坐标系): (" << fixed << setprecision(3)
           << detection.translationVector.at<double>(0, 0) << ", "
           << detection.translationVector.at<double>(1, 0) << ", "
           << detection.translationVector.at<double>(2, 0) << ") m\n";

      // 清理位姿结构体
      matd_destroy(pose.R);
      matd_destroy(pose.t);
    }
  }

  // 清理资源
  image_u8_destroy(img_u8);
  apriltag_detections_destroy(detections_array);
  apriltag_detector_destroy(td);

  cout << "\n";
  return detections;
}

/**
 * @brief 将相机坐标系的位姿变换到世界坐标系
 * @param cameraDetection 相机坐标系中的位姿
 * @param cameraExtrinsics 相机的外参
 * @return 世界坐标系中的位姿
 *
 * 变换关系：
 *   P_world = R_c2w * P_camera + t_c2w
 *
 *   其中：
 *   - P_world: 点在世界坐标系中的坐标
 *   - R_c2w: 相机坐标系到世界坐标系的旋转矩阵
 *   - P_camera: 点在相机坐标系中的坐标
 *   - t_c2w: 相机坐标系原点在世界坐标系中的位置
 */
AprilTagWorldPose transformToWorldCoordinates(
    const AprilTagDetection& cameraDetection,
    const CameraExtrinsics& cameraExtrinsics) {
  AprilTagWorldPose worldPose;
  worldPose.tagId = cameraDetection.tagId;

  // 获取相机到世界的变换
  // 外参给定的是世界到相机的变换 (R_w2c, t_w2c)
  Mat R_w2c = cameraExtrinsics.rotationMatrix;     // 世界→相机 旋转
  Mat t_w2c = cameraExtrinsics.translationVector;  // 世界→相机 平移 (mm)

  // 反向计算得到相机到世界的变换
  // R_c2w = R_w2c^T (旋转矩阵的逆等于转置)
  // t_c2w = -R_c2w * t_w2c
  Mat R_c2w = R_w2c.t();
  Mat t_c2w = -R_c2w * t_w2c;

  // 标签在相机坐标系中的位姿
  Mat R_cam2tag = cameraDetection.rotationMatrix;  // 相机→标签 旋转
  Mat t_cam2tag = cameraDetection.translationVector.clone() *
                  1000;  // 相机→标签 平移 (转换为 mm)

  // 标签在世界坐标系中的旋转：R_world2tag = R_c2w * R_cam2tag
  worldPose.rotationMatrix = R_c2w * R_cam2tag;

  // 标签在世界坐标系中的平移：t_world2tag = R_c2w * t_cam2tag + t_c2w
  worldPose.translationVector = R_c2w * t_cam2tag + t_c2w;

  // 转换为旋转向量
  Rodrigues(worldPose.rotationMatrix, worldPose.rotationVector);

  worldPose.isValid = true;

  return worldPose;
}

/**
 * @brief 保存检测结果到 YAML 文件
 */
void saveResultsToYAML(const string& filename,
                       const vector<AprilTagDetection>& cameraDetections,
                       const vector<AprilTagWorldPose>& worldPoses) {
  cout << "[步骤] 保存结果到文件: " << filename << "\n";

  FileStorage fs(filename, FileStorage::WRITE);

  if (!fs.isOpened()) {
    cerr << "[ERROR] 无法打开文件进行写入: " << filename << endl;
    return;
  }

  fs << "detection_results"
     << "[";

  for (size_t i = 0; i < worldPoses.size(); ++i) {
    fs << "{";
    fs << "tag_id" << worldPoses[i].tagId;
    fs << "rotation_matrix_world" << worldPoses[i].rotationMatrix;
    fs << "rotation_vector_world" << worldPoses[i].rotationVector;
    fs << "translation_vector_world_mm" << worldPoses[i].translationVector;
    fs << "}";
  }

  fs << "]";

  fs.release();

  cout << "[成功] 结果已保存\n\n";
}

/**
 * @brief 打印检测和变换结果
 */
void printResults(const vector<AprilTagDetection>& cameraDetections,
                  const vector<AprilTagWorldPose>& worldPoses) {
  cout << "\n════════════════════════════════════════════════════════════\n";
  cout << "                    AprilTag 识别结果\n";
  cout << "════════════════════════════════════════════════════════════\n\n";

  if (worldPoses.empty()) {
    cout << "[信息] 未检测到任何 AprilTag\n";
    cout << "════════════════════════════════════════════════════════════\n\n";
    return;
  }

  for (size_t i = 0; i < worldPoses.size(); ++i) {
    const auto& worldPose = worldPoses[i];
    const auto& cameraDet = cameraDetections[i];

    cout << "─ AprilTag #" << worldPose.tagId << "\n\n";

    // 相机坐标系信息
    cout << "  [相机坐标系]\n";
    cout << "    位置: (" << fixed << setprecision(2)
         << cameraDet.translationVector.at<double>(0, 0) * 1000 << ", "
         << cameraDet.translationVector.at<double>(1, 0) * 1000 << ", "
         << cameraDet.translationVector.at<double>(2, 0) * 1000 << ") mm\n";

    double angle_cam = norm(cameraDet.rotationVector);
    double angleDeg_cam = angle_cam * 180.0 / M_PI;
    cout << "    旋转角度: " << fixed << setprecision(2) << angleDeg_cam
         << "°\n\n";

    // 世界坐标系信息
    cout << "  [世界坐标系]\n";
    cout << "    位置 (右前上):\n";
    cout << "      X (向右): " << fixed << setprecision(2)
         << worldPose.translationVector.at<double>(0, 0) << " mm\n";
    cout << "      Y (向前): " << worldPose.translationVector.at<double>(1, 0)
         << " mm\n";
    cout << "      Z (向上): " << worldPose.translationVector.at<double>(2, 0)
         << " mm\n\n";

    cout << "    旋转矩阵 (世界→标签):\n";
    for (int r = 0; r < 3; ++r) {
      cout << "      [";
      for (int c = 0; c < 3; ++c) {
        cout << fixed << setprecision(4)
             << worldPose.rotationMatrix.at<double>(r, c);
        if (c < 2) cout << " ";
      }
      cout << "]\n";
    }

    double angle_world = norm(worldPose.rotationVector);
    double angleDeg_world = angle_world * 180.0 / M_PI;
    cout << "\n    旋转角度: " << fixed << setprecision(2) << angleDeg_world
         << "°\n";
    cout << "    旋转向量: [" << fixed << setprecision(4)
         << worldPose.rotationVector.at<double>(0, 0) << ", "
         << worldPose.rotationVector.at<double>(1, 0) << ", "
         << worldPose.rotationVector.at<double>(2, 0) << "]\n\n";
  }

  cout << "════════════════════════════════════════════════════════════\n\n";
}

/**
 * @brief 可视化检测结果
 */
void visualizeDetections(const Mat& image,
                         const vector<AprilTagDetection>& detections,
                         const Mat& cameraMatrix, const Mat& distCoeffs) {
  Mat display = image.clone();

  // 缩小1倍（尺寸减半）
  Mat displayResized;
  resize(display, displayResized, Size(display.cols / 2, display.rows / 2));

  // 调整检测结果中的坐标和相机矩阵（缩放因子为 0.5）
  float scale = 0.5f;
  Mat cameraMatrixScaled = cameraMatrix.clone();
  cameraMatrixScaled.at<double>(0, 0) *= scale;  // fx
  cameraMatrixScaled.at<double>(1, 1) *= scale;  // fy
  cameraMatrixScaled.at<double>(0, 2) *= scale;  // cx
  cameraMatrixScaled.at<double>(1, 2) *= scale;  // cy

  for (const auto& detection : detections) {
    // 缩放检测结果中的2D坐标
    vector<Point2f> scaledCorners;
    for (const auto& corner : detection.corners) {
      scaledCorners.push_back(Point2f(corner.x * scale, corner.y * scale));
    }

    Point2f scaledCenter(detection.center.x * scale,
                         detection.center.y * scale);

    // 绘制标记的边界
    for (size_t i = 0; i < scaledCorners.size(); ++i) {
      Point p1 = scaledCorners[i];
      Point p2 = scaledCorners[(i + 1) % scaledCorners.size()];
      line(displayResized, p1, p2, Scalar(0, 255, 0), 2);
    }

    // 绘制中心点
    circle(displayResized, scaledCenter, 3, Scalar(0, 0, 255), -1);

    // 绘制标签 ID
    putText(displayResized, "ID: " + to_string(detection.tagId),
            Point(scaledCenter.x - 20, scaledCenter.y - 15),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

    // 绘制坐标系轴
    vector<Point3f> axisPoints;
    axisPoints.push_back(Point3f(0, 0, 0));     // 原点
    axisPoints.push_back(Point3f(0.05, 0, 0));  // X 轴 (红)
    axisPoints.push_back(Point3f(0, 0.05, 0));  // Y 轴 (绿)
    axisPoints.push_back(Point3f(0, 0, 0.05));  // Z 轴 (蓝)

    vector<Point2f> projectedPoints;
    projectPoints(axisPoints, detection.rotationVector,
                  detection.translationVector, cameraMatrixScaled, distCoeffs,
                  projectedPoints);

    // X 轴 - 红
    line(displayResized, projectedPoints[0], projectedPoints[1],
         Scalar(0, 0, 255), 2);
    putText(displayResized, "X", projectedPoints[1], FONT_HERSHEY_SIMPLEX, 0.4,
            Scalar(0, 0, 255), 1);

    // Y 轴 - 绿
    line(displayResized, projectedPoints[0], projectedPoints[2],
         Scalar(0, 255, 0), 2);
    putText(displayResized, "Y", projectedPoints[2], FONT_HERSHEY_SIMPLEX, 0.4,
            Scalar(0, 255, 0), 1);

    // Z 轴 - 蓝
    line(displayResized, projectedPoints[0], projectedPoints[3],
         Scalar(255, 0, 0), 2);
    putText(displayResized, "Z", projectedPoints[3], FONT_HERSHEY_SIMPLEX, 0.4,
            Scalar(255, 0, 0), 1);
  }

  // 显示图像（带可移动窗口）
  namedWindow("AprilTag Detection", WINDOW_NORMAL | WINDOW_KEEPRATIO);
  moveWindow("AprilTag Detection", 100, 100);  // 移动窗口到 (100, 100) 位置
  imshow("AprilTag Detection", displayResized);

  cout << "[信息] 显示识别结果 (按任意键关闭)\n\n";
  waitKey(0);
  destroyWindow("AprilTag Detection");
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
  if (argc < 5) {
    printUsage(argv[0]);
    return -1;
  }

  string imageFile = argv[1];
  string intrinsicFile = argv[2];
  string extrinsicFile = argv[3];
  double tagSize = atof(argv[4]);

  // 从输入图像路径生成输出文件名
  // 提取图像文件名（不含路径和扩展名）
  size_t lastSlash = imageFile.find_last_of("/\\");
  string imageDir =
      (lastSlash != string::npos) ? imageFile.substr(0, lastSlash) : ".";
  string imageFilename =
      (lastSlash != string::npos) ? imageFile.substr(lastSlash + 1) : imageFile;
  size_t lastDot = imageFilename.find_last_of(".");
  string imageBasename = (lastDot != string::npos)
                             ? imageFilename.substr(0, lastDot)
                             : imageFilename;

  // 生成输出文件名：result_[图像名].yaml，保存在图像所在目录
  string outputFile = imageDir + "/result_" + imageBasename + ".yaml";

  apriltag_family_t* tf = tagStandard41h12_create();  // 默认使用 tag41h12
  std::cout << "[信息] 默认使用 tagStandard41h12 标签族\n\n";
  bool visualize = false;

  // 解析可选参数
  for (int i = 5; i < argc; ++i) {
    string arg = argv[i];

    if (arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--tag-family" && i + 1 < argc) {
      string family = argv[++i];

      // 销毁旧的标签族
      if (tf != nullptr) {
        tagStandard41h12_destroy(tf);
      }

      if (family == "16h5") {
        tf = tag16h5_create();
        cout << "[信息] 使用 tag16h5 标签族\n";
      } else if (family == "25h9") {
        tf = tag25h9_create();
        cout << "[信息] 使用 tag25h9 标签族\n";
      } else if (family == "36h10") {
        tf = tag36h10_create();
        cout << "[信息] 使用 tag36h10 标签族\n";
      } else if (family == "36h11") {
        tf = tag36h11_create();
        cout << "[信息] 使用 tag36h11 标签族\n";
      } else if (family == "circle21h7") {
        tf = tagCircle21h7_create();
        cout << "[信息] 使用 tagCircle21h7 标签族\n";
      } else if (family == "circle49h12") {
        tf = tagCircle49h12_create();
        cout << "[信息] 使用 tagCircle49h12 标签族\n";
      } else if (family == "custom48h12") {
        tf = tagCustom48h12_create();
        cout << "[信息] 使用 tagCustom48h12 标签族\n";
      } else if (family == "41h12") {
        tf = tagStandard41h12_create();
        cout << "[信息] 使用 tagStandard41h12 标签族\n";
      } else if (family == "52h13") {
        tf = tagStandard52h13_create();
        cout << "[信息] 使用 tagStandard52h13 标签族\n";
      } else {
        cerr << "[ERROR] 未知的标签族: " << family << endl;
        return -1;
      }
    } else if (arg == "--output" && i + 1 < argc) {
      outputFile = argv[++i];
    } else if (arg == "--visualize") {
      visualize = true;
    }
  }

  cout << "\n╔════════════════════════════════════════════════════════════╗\n";
  cout << "║     AprilTag 识别和位姿求解程序 v2.0 (apriltag3)         ║\n";
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

  cout << "[成功] 读取相机内参: " << intrinsicFile << "\n";
  cout << "  相机矩阵:\n" << intrinsics.cameraMatrix << "\n";
  cout << "  畸变系数:\n" << intrinsics.distortionCoeffs.t() << "\n\n";

  // ====================================================================
  // 步骤 2: 读取相机外参
  // ====================================================================
  cout << "[步骤 2] 读取相机外参...\n";
  cout << "════════════════════════════════════════════════════════════\n";
  CameraExtrinsics extrinsics = readExtrinsicsFromYAML(extrinsicFile);

  if (!extrinsics.isValid) {
    cerr << "[ERROR] 读取外参失败\n";
    return -1;
  }

  cout << "[成功] 读取相机外参: " << extrinsicFile << "\n";
  cout << "  旋转矩阵:\n" << extrinsics.rotationMatrix << "\n";
  cout << "  平移向量 (mm): " << extrinsics.translationVector.t() << "\n\n";

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
  cout << "  图像尺寸: " << image.cols << " x " << image.rows << " 像素\n\n";

  // ====================================================================
  // 步骤 4: 检测 AprilTag
  // ====================================================================
  vector<AprilTagDetection> detections = detectAprilTags(
      image, tagSize, intrinsics.cameraMatrix, intrinsics.distortionCoeffs, tf);

  if (detections.empty()) {
    cout << "[信息] 未检测到任何 AprilTag\n";
    if (tf != nullptr) {
      tagStandard41h12_destroy(tf);
    }
    return 0;
  }

  // ====================================================================
  // 步骤 5: 坐标系变换（相机→世界）
  // ====================================================================
  cout << "[步骤 4] 坐标系变换 (相机→世界)...\n";
  cout << "════════════════════════════════════════════════════════════\n";

  vector<AprilTagWorldPose> worldPoses;
  for (const auto& detection : detections) {
    AprilTagWorldPose worldPose =
        transformToWorldCoordinates(detection, extrinsics);
    worldPoses.push_back(worldPose);
    cout << "  [成功] 标签 ID " << detection.tagId << " 已变换到世界坐标系\n";
  }

  cout << "════════════════════════════════════════════════════════════\n\n";

  // ====================================================================
  // 步骤 6: 显示结果
  // ====================================================================
  printResults(detections, worldPoses);

  // ====================================================================
  // 步骤 7: 保存结果
  // ====================================================================
  saveResultsToYAML(outputFile, detections, worldPoses);

  // ====================================================================
  // 步骤 8: 可视化
  // ====================================================================
  if (visualize) {
    cout << "[步骤 5] 可视化检测结果...\n";
    cout << "════════════════════════════════════════════════════════════\n";
    visualizeDetections(image, detections, intrinsics.cameraMatrix,
                        intrinsics.distortionCoeffs);
    cout << "════════════════════════════════════════════════════════════\n\n";
  }

  // 清理资源
  if (tf != nullptr) {
    tagStandard41h12_destroy(tf);
  }

  cout << "[完成] AprilTag 识别和位姿求解完毕\n";
  cout << "════════════════════════════════════════════════════════════\n\n";

  return 0;
}
