#pragma once

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

// Bundle Adjustment相关的结构体和函数声明
#include <ceres/ceres.h>
#include <ceres/rotation.h>

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
  Mat rotationMatrix;        // 方法1: estimate_tag_pose() 结果
  Mat rotationVector;        // 方法1: estimate_tag_pose() 结果
  Mat translationVector;     // 方法1: estimate_tag_pose() 结果
  Mat pnpRotationMatrix;     // 方法2: PnP 结果 - 旋转矩阵
  Mat pnpRotationVector;     // 方法2: PnP 结果 - 旋转向量
  Mat pnpTranslationVector;  // 方法2: PnP 结果 - 平移向量
  Mat baRotationMatrix;      // 方法3: Bundle Adjustment 结果 - 旋转矩阵
  Mat baRotationVector;      // 方法3: Bundle Adjustment 结果 - 旋转向量
  Mat baTranslationVector;   // 方法3: Bundle Adjustment 结果 - 平移向量
  Mat rayTranslationVector;  // 方法4: Ray-Plane Intersection 结果 - 平移向量
  bool isValid;

  AprilTagDetection() : tagId(-1), isValid(false) {
    rotationMatrix = Mat::eye(3, 3, CV_64F);
    rotationVector = Mat::zeros(3, 1, CV_64F);
    translationVector = Mat::zeros(3, 1, CV_64F);
    pnpRotationMatrix = Mat::eye(3, 3, CV_64F);
    pnpRotationVector = Mat::zeros(3, 1, CV_64F);
    pnpTranslationVector = Mat::zeros(3, 1, CV_64F);
    baRotationMatrix = Mat::eye(3, 3, CV_64F);
    baRotationVector = Mat::zeros(3, 1, CV_64F);
    baTranslationVector = Mat::zeros(3, 1, CV_64F);
    rayTranslationVector = Mat::zeros(3, 1, CV_64F);
  }
};

// 滑动窗口结构：用于积累多帧检测结果
struct SlidingWindow {
  int windowSize;
  vector<vector<Point2f>> allCorners;  // 滑动窗口内的所有角点 (每帧4个点)
  vector<Point2f> allCenters;          // 滑动窗口内的所有中心点

  SlidingWindow(int size) : windowSize(size) {}

  // 添加新的检测结果到滑动窗口
  void addDetection(const vector<Point2f>& corners, const Point2f& center);

  // 获取滑动窗口内的所有检测结果
  // tagSize: AprilTag标签的实际尺寸（米）
  void getWindowData(vector<Point2f>& allImagePoints);

  // 检查窗口是否满
  bool isFull() const { return allCorners.size() == windowSize; }
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
void printUsage(const char* programName);

/**
 * @brief 从 YAML 文件读取相机内参
 */
CameraIntrinsics readIntrinsicsFromYAML(const string& filename);

/**
 * @brief 从 YAML 文件读取相机外参
 */
CameraExtrinsics readExtrinsicsFromYAML(const string& filename);

/**
 * @brief 检测图像中的 AprilTag
 */
AprilTagDetection detectAprilTags(const Mat& image, double tagSize,
                                  const Mat& cameraMatrix,
                                  const Mat& distCoeffs);

/**
 * @brief 将相机坐标系的位姿变换到世界坐标系
 * @param usePnP 是否使用PnP结果进行位姿变换
 * @param useBA 是否使用Bundle Adjustment结果进行位姿变换
 */
AprilTagWorldPose transformToWorldCoordinates(
    const AprilTagDetection& cameraDetection,
    const CameraExtrinsics& cameraExtrinsics,
    bool useBA = false);  // 默认使用原始的estimate_tag_pose结果

/**
 * @brief 使用滑动窗口的PnP位姿估计
 */
AprilTagDetection detectAprilTagsWithSlidingWindow(
    const Mat& image, double tagSize, const Mat& cameraMatrix,
    const Mat& distCoeffs, SlidingWindow& window, double zConstraint,
    int poseMethod);

/**
 * @brief 方法4: 使用射线-平面求交法计算 AprilTag 在世界坐标系中的位置
 */
AprilTagWorldPose rayPlaneIntersectionMethod(
    const AprilTagDetection& cameraDetection,
    const CameraIntrinsics& cameraIntrinsics,
    const CameraExtrinsics& cameraExtrinsics, double zConstraint);

// 辅助函数：将旋转矩阵转换为四元数
Vec4d rotationMatrixToQuaternion(const Mat& rotationMatrix);

/**
 * @brief 保存结果为 TUM 格式文件
 */
void saveResultsToTXT(
    const string& filename,
    const vector<pair<string, vector<AprilTagWorldPose>>>& allResults);

// 辅助函数：自然排序比较器
bool naturalCompare(const string& a, const string& b);

/**
 * @brief 列出文件夹内所有 .bmp 和 .jpg 图像文件
 */
vector<string> listImageFiles(const string& folderPath);

/**
 * @brief 绘制 AprilTag 在 XOY 世界坐标系的轨迹俯视图（EVO 风格）
 */
void visualizeTrajectory(
    const vector<pair<string, vector<AprilTagWorldPose>>>& allResults,
    const string& outputFile);

// Observations for one frame: 4 corners (order must match object points
// order)
struct FrameObservation {
  // image points for corners (e.g., order: [rt, lt, lb, rb] or as your detector
  // uses)
  std::vector<cv::Point2d> corners_px;  // expected size 4
};

// Simple helper to create tag corner 3D points in tag-local coords
inline std::vector<cv::Point3d> MakeTagCorners3D(double tag_size_m) {
  double hs = tag_size_m / 2.0;
  // Match the order used by your detector. Ensure consistent order.
  // Here we follow: 0: right-top, 1: left-top, 2: left-bottom, 3: right -
  // bottom
  std::vector<cv::Point3d> pts = {
      {hs, hs, 0.0}, {-hs, hs, 0.0}, {-hs, -hs, 0.0}, {hs, -hs, 0.0}};
  return pts;
}

// /**
//  * @brief 执行Bundle Adjustment优化
//  */
bool bundleAdjustmentTagWindow(
    const std::vector<FrameObservation>& observations,
    const std::vector<std::array<double, 6>>& poses_initial,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double tag_size_m,
    double desired_tag_z, double height_weight,
    std::vector<std::array<double, 6>>& poses_optimized);
