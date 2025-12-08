#pragma once

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// 数据结构（与 apriltag_detector.cpp 保持一致）
struct CameraIntrinsics {
  cv::Mat cameraMatrix;
  cv::Mat distortionCoeffs;
  int imageWidth;
  int imageHeight;
  bool isValid;

  CameraIntrinsics() : isValid(false), imageWidth(0), imageHeight(0) {
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distortionCoeffs = cv::Mat::zeros(5, 1, CV_64F);
  }
};

struct CameraExtrinsics {
  cv::Mat rotationMatrix;
  cv::Mat rotationVector;
  cv::Mat translationVector;
  bool isValid;

  CameraExtrinsics() : isValid(false) {
    rotationMatrix = cv::Mat::eye(3, 3, CV_64F);
    rotationVector = cv::Mat::zeros(3, 1, CV_64F);
    translationVector = cv::Mat::zeros(3, 1, CV_64F);
  }
};

CameraIntrinsics readIntrinsicsFromYAML(const std::string& filename);

/**
 * @brief 从 YAML 文件读取相机外参
 */
CameraExtrinsics readExtrinsicsFromYAML(const std::string& filename);
