/**
 * @file apriltag_detector.cpp
 * @brief AprilTag 识别和位姿求解程序
 *
 * @details
 *   该程序实现了以下功能：
 *   1. 读取相机内参文件（YAML格式）
 *   2. 读取相机的外参文件（相对世界坐标系）
 *   3. 识别图像中的 AprilTag 标记
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
 * <tag_size>
 *   ./apriltag_detector <image_file> <intrinsic_yaml> <extrinsic_yaml>
 * <tag_size> [options]
 *
 * @example
 *   ./apriltag_detector photo.jpg camera_calib.yaml extrinsic.yaml 0.15
 *   ./apriltag_detector photo.jpg camera_calib.yaml extrinsic.yaml 0.15
 * --output result.yaml
 *   ./apriltag_detector photo.jpg camera_calib.yaml extrinsic.yaml 0.15
 * --visualize
 */

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/aruco.hpp>
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
  Mat distortionCoeffs;  ///< 畸变系数
  int imageWidth;        ///< 图像宽度
  int imageHeight;       ///< 图像高度
  bool isValid;          ///< 是否有效

  CameraIntrinsics() : isValid(false), imageWidth(0), imageHeight(0) {
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distortionCoeffs = Mat::zeros(5, 1, CV_64F);
  }
};

/// 相机外参结构体
struct CameraExtrinsics {
  Mat rotationMatrix;     ///< 3×3 旋转矩阵 (世界→相机)
  Mat rotationVector;     ///< 3×1 旋转向量
  Mat translationVector;  ///< 3×1 平移向量 (单位: mm)
  bool isValid;           ///< 是否有效

  CameraExtrinsics() : isValid(false) {
    rotationMatrix = Mat::eye(3, 3, CV_64F);
    rotationVector = Mat::zeros(3, 1, CV_64F);
    translationVector = Mat::zeros(3, 1, CV_64F);
  }
};

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
  cout << "          AprilTag 识别和位姿求解程序 (基于相机标定)\n";
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
  cout << "                         支持: 36h11, 25h9, 16h5 (默认: 36h11)\n";
  cout << "  --output <file>        输出结果文件 (YAML格式)\n";
  cout << "  --visualize            显示识别结果和坐标系\n";
  cout << "  --help                 显示此帮助信息\n\n";
  cout << "说明:\n";
  cout << "  - 本程序使用 OpenCV 的 ArUco 模块识别 AprilTag\n";
  cout << "  - AprilTag 在 ArUco 字典中的对应关系：\n";
  cout << "    * DICT_APRILTAG_36h11 ← tag36h11（推荐）\n";
  cout << "    * DICT_APRILTAG_25h9  ← tag25h9\n";
  cout << "    * DICT_APRILTAG_16h5  ← tag16h5\n";
  cout << "  - 输入图像应为未去畸变的原始图像\n";
  cout << "  - 标签尺寸应为标签的物理宽度（从内边缘到内边缘）\n\n";
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

    if (!fs["image_width"].empty()) {
      intrinsics.imageWidth = (int)fs["image_width"];
    }
    if (!fs["image_height"].empty()) {
      intrinsics.imageHeight = (int)fs["image_height"];
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
 * @param image 输入图像
 * @param tagSize AprilTag 尺寸（米）
 * @param cameraMatrix 相机矩阵
 * @param distCoeffs 畸变系数
 * @param dictId ArUco 字典 ID
 * @return 检测到的标签列表
 */
vector<AprilTagDetection> detectAprilTags(const Mat& image, double tagSize,
                                          const Mat& cameraMatrix,
                                          const Mat& distCoeffs, int dictId) {
  vector<AprilTagDetection> detections;

  cout << "[步骤] 检测 AprilTag 标记...\n";
  cout << "════════════════════════════════════════════════════════════\n";
  cout << "  标签尺寸: " << fixed << setprecision(4) << tagSize << " m\n";

  // 获取 ArUco 字典 (使用 OpenCV 4.3+ 的 API)
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(dictId);

  // 转换为灰度图像
  Mat gray;
  if (image.channels() == 3) {
    cvtColor(image, gray, COLOR_BGR2GRAY);
  } else {
    gray = image.clone();
  }

  // 检测标记 (使用更兼容的 API)
  vector<int> ids;
  vector<vector<Point2f>> corners;
  cv::aruco::detectMarkers(gray, dict, corners, ids);

  cout << "  检测到 " << ids.size() << " 个标记\n";

  if (ids.empty()) {
    cout << "  [警告] 未检测到任何 AprilTag\n";
    cout << "════════════════════════════════════════════════════════════\n";
    return detections;
  }

  // 为每个检测到的标记估计位姿
  vector<Mat> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, tagSize, cameraMatrix,
                                       distCoeffs, rvecs, tvecs);

  cout << "  [" << ids.size() << "] 标记检测成功\n";

  for (size_t i = 0; i < ids.size(); ++i) {
    AprilTagDetection detection;
    detection.tagId = ids[i];
    detection.corners = corners[i];

    // 计算中心点
    Point2f center(0, 0);
    for (const auto& corner : corners[i]) {
      center += corner;
    }
    center *= 0.25f;
    detection.center = center;

    // 旋转向量和平移向量
    detection.rotationVector = rvecs[i].clone();
    detection.translationVector = tvecs[i].clone();

    // 转换为旋转矩阵
    Rodrigues(detection.rotationVector, detection.rotationMatrix);

    detection.isValid = true;
    detections.push_back(detection);

    cout << "  ├─ 标记 ID: " << detection.tagId << "\n";
    cout << "  │  位置 (相机坐标系): (" << fixed << setprecision(3)
         << detection.translationVector.at<double>(0, 0) * 1000 << ", "
         << detection.translationVector.at<double>(1, 0) * 1000 << ", "
         << detection.translationVector.at<double>(2, 0) * 1000 << ") mm\n";
  }

  cout << "════════════════════════════════════════════════════════════\n\n";

  return detections;
}

/**
 * @brief 将相机坐标系的位姿变换到世界坐标系
 * @param cameraDetection 相机坐标系中的位姿
 * @param cameraExtrinsics 相机的外参
 * @return 世界坐标系中的位姿
 *
 * 变换关系：
 *   T_world_tag = T_world_camera * T_camera_tag
 *
 *   其中：
 *   - T_world_tag: 标签在世界坐标系中的位姿
 *   - T_world_camera: 相机在世界坐标系中的位姿（由外参给出）
 *   - T_camera_tag: 标签在相机坐标系中的位姿
 */
AprilTagWorldPose transformToWorldCoordinates(
    const AprilTagDetection& cameraDetection,
    const CameraExtrinsics& cameraExtrinsics) {
  AprilTagWorldPose worldPose;
  worldPose.tagId = cameraDetection.tagId;

  // 标签在相机坐标系中的位姿（单位转换为 mm）
  Mat tag_R_cam = cameraDetection.rotationMatrix.t();  // 相机→标签的反向旋转
  Mat tag_t_cam =
      cameraDetection.translationVector.clone() * 1000;  // 转换为 mm
  tag_t_cam = -tag_R_cam * tag_t_cam;                    // 反向平移

  // 世界到相机的变换矩阵
  // T_w2c = [R_w2c | t_w2c]
  Mat R_w2c = cameraExtrinsics.rotationMatrix;     // 世界→相机
  Mat t_w2c = cameraExtrinsics.translationVector;  // 世界→相机的平移

  // 计算相机到世界的变换（反向）
  // T_c2w = [R_c2w | t_c2w]
  Mat R_c2w = R_w2c.t();       // 相机→世界
  Mat t_c2w = -R_c2w * t_w2c;  // 相机→世界的平移

  // 世界到标签的变换 = 相机到世界的变换 * 相机到标签的变换
  // T_w2tag = T_c2w * T_c2tag
  worldPose.rotationMatrix = R_c2w * tag_R_cam;
  worldPose.translationVector = R_c2w * tag_t_cam + t_c2w;

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

  for (const auto& detection : detections) {
    // 绘制标记的边界
    for (size_t i = 0; i < detection.corners.size(); ++i) {
      Point p1 = detection.corners[i];
      Point p2 = detection.corners[(i + 1) % detection.corners.size()];
      line(display, p1, p2, Scalar(0, 255, 0), 3);
    }

    // 绘制中心点
    circle(display, detection.center, 5, Scalar(0, 0, 255), -1);

    // 绘制标签 ID
    putText(display, "ID: " + to_string(detection.tagId),
            Point(detection.center.x - 30, detection.center.y - 20),
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

    // 绘制坐标系轴
    vector<Point3f> axisPoints;
    axisPoints.push_back(Point3f(0, 0, 0));     // 原点
    axisPoints.push_back(Point3f(0.05, 0, 0));  // X 轴 (红)
    axisPoints.push_back(Point3f(0, 0.05, 0));  // Y 轴 (绿)
    axisPoints.push_back(Point3f(0, 0, 0.05));  // Z 轴 (蓝)

    vector<Point2f> projectedPoints;
    projectPoints(axisPoints, detection.rotationVector,
                  detection.translationVector, cameraMatrix, distCoeffs,
                  projectedPoints);

    // X 轴 - 红
    line(display, projectedPoints[0], projectedPoints[1], Scalar(0, 0, 255), 2);
    putText(display, "X", projectedPoints[1], FONT_HERSHEY_SIMPLEX, 0.5,
            Scalar(0, 0, 255), 2);

    // Y 轴 - 绿
    line(display, projectedPoints[0], projectedPoints[2], Scalar(0, 255, 0), 2);
    putText(display, "Y", projectedPoints[2], FONT_HERSHEY_SIMPLEX, 0.5,
            Scalar(0, 255, 0), 2);

    // Z 轴 - 蓝
    line(display, projectedPoints[0], projectedPoints[3], Scalar(255, 0, 0), 2);
    putText(display, "Z", projectedPoints[3], FONT_HERSHEY_SIMPLEX, 0.5,
            Scalar(255, 0, 0), 2);
  }

  // 显示图像
  namedWindow("AprilTag Detection", WINDOW_NORMAL);
  imshow("AprilTag Detection", display);

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

  string outputFile = "apriltag_result.yaml";
  int dictId = cv::aruco::DICT_APRILTAG_36h11;
  bool visualize = false;

  // 解析可选参数
  for (int i = 5; i < argc; ++i) {
    string arg = argv[i];

    if (arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--tag-family" && i + 1 < argc) {
      string family = argv[++i];
      if (family == "36h11") {
        dictId = cv::aruco::DICT_APRILTAG_36h11;
      } else if (family == "25h9") {
        dictId = cv::aruco::DICT_APRILTAG_25h9;
      } else if (family == "16h5") {
        dictId = cv::aruco::DICT_APRILTAG_16h5;
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
  cout << "║       AprilTag 识别和位姿求解程序 v1.0                   ║\n";
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
  vector<AprilTagDetection> detections =
      detectAprilTags(image, tagSize, intrinsics.cameraMatrix,
                      intrinsics.distortionCoeffs, dictId);

  if (detections.empty()) {
    cout << "[信息] 未检测到任何 AprilTag\n";
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

  cout << "[完成] AprilTag 识别和位姿求解完毕\n";
  cout << "════════════════════════════════════════════════════════════\n\n";

  return 0;
}
