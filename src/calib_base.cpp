#include "calib_base.h"

CameraIntrinsics readIntrinsicsFromYAML(const std::string& filename) {
  CameraIntrinsics intrinsics;

  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    std::cerr << "[ERROR] 无法打开内参文件: " << filename << std::endl;
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

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] 读取内参 YAML 文件异常: " << e.what() << std::endl;
    intrinsics.isValid = false;
  }

  fs.release();
  return intrinsics;
}

CameraExtrinsics readExtrinsicsFromYAML(const std::string& filename) {
  CameraExtrinsics extrinsics;

  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    std::cerr << "[ERROR] 无法打开外参文件: " << filename << std::endl;
    return extrinsics;
  }

  try {
    fs["rotation_matrix"] >> extrinsics.rotationMatrix;
    fs["rotation_vector"] >> extrinsics.rotationVector;
    fs["translation_vector"] >> extrinsics.translationVector;

    extrinsics.isValid = true;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] 读取外参 YAML 文件异常: " << e.what() << std::endl;
    extrinsics.isValid = false;
  }

  fs.release();
  return extrinsics;
}
