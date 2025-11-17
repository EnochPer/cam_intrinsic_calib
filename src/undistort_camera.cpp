/**
 * @file undistort_camera.cpp
 * @brief 图像去畸变程序 - 读取相机标定结果，对图像进行去畸变处理
 *
 * @details
 *   该程序实现了以下功能：
 *   1. 读取相机标定的内参 YAML 文件（相机矩阵和畸变系数）
 *   2. 遍历指定目录下的所有图像
 *   3. 对每张图像进行去畸变处理
 *   4. 将去畸变后的图像保存到输出目录
 *
 * @usage
 *   ./undistort_camera <calibration_yaml> <input_directory> <output_directory>
 *   ./undistort_camera <calibration_yaml> <input_directory>  # 默认在原目录生成
 * undistorted 子目录
 *
 * @example
 *   ./undistort_camera camera_calibration.yaml ~/Pictures/raw_images
 * ~/Pictures/undistorted
 *   ./undistort_camera camera_calibration.yaml ~/Pictures/raw_images
 */

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace cv;
using namespace std;

// ============================================================================
// 配置参数
// ============================================================================

/// 调试开关
static const bool DEBUG_UNDISTORT = true;  ///< 是否输出详细的去畸变处理信息

// ============================================================================
// 辅助函数
// ============================================================================

/**
 * @brief 打印程序用法
 */
void printUsage(const char* programName) {
  cout << "\n========================================\n";
  cout << "相机图像去畸变程序\n";
  cout << "========================================\n\n";
  cout << "用法:\n";
  cout << "  " << programName
       << " <calibration_yaml> <input_directory> [output_directory]\n\n";
  cout << "参数:\n";
  cout << "  <calibration_yaml>     相机标定结果 YAML 文件\n";
  cout << "  <input_directory>      输入图像目录\n";
  cout << "  [output_directory]     输出目录 (默认: "
          "input_directory/undistorted)\n\n";
  cout << "选项:\n";
  cout << "  --help                 显示此帮助信息\n\n";
  cout << "示例:\n";
  cout << "  " << programName
       << " camera_calibration.yaml ~/Pictures/raw ~/Pictures/undistorted\n";
  cout << "  " << programName << " camera_calibration.yaml ~/Pictures/raw\n";
  cout << "\n========================================\n\n";
}

/**
 * @brief 从 YAML 文件读取相机标定结果
 * @param calibrationFile 标定结果文件路径
 * @param cameraMatrix 输出的相机矩阵
 * @param distortionCoeffs 输出的畸变系数
 * @return 是否读取成功
 */
bool loadCalibrationData(const string& calibrationFile, Mat& cameraMatrix,
                         Mat& distortionCoeffs) {
  cout << "[步骤] 读取标定数据...\n";
  cout << "========================================\n";

  // 检查文件是否存在
  if (!fs::exists(calibrationFile)) {
    cerr << "[ERROR] 标定文件不存在: " << calibrationFile << endl;
    return false;
  }

  FileStorage fs(calibrationFile, FileStorage::READ);
  if (!fs.isOpened()) {
    cerr << "[ERROR] 无法打开标定文件: " << calibrationFile << endl;
    return false;
  }

  try {
    // 读取相机矩阵
    cameraMatrix = fs["camera_matrix"].mat();
    if (cameraMatrix.empty()) {
      cerr << "[ERROR] 无法读取相机矩阵\n";
      fs.release();
      return false;
    }

    // 读取畸变系数
    distortionCoeffs = fs["distortion_coefficients"].mat();
    if (distortionCoeffs.empty()) {
      cerr << "[ERROR] 无法读取畸变系数\n";
      fs.release();
      return false;
    }

    fs.release();

    cout << "[成功] 标定数据读取完成\n";
    cout << "  相机矩阵:\n";
    cout << "    fx = " << cameraMatrix.at<double>(0, 0) << " px\n";
    cout << "    fy = " << cameraMatrix.at<double>(1, 1) << " px\n";
    cout << "    cx = " << cameraMatrix.at<double>(0, 2) << " px\n";
    cout << "    cy = " << cameraMatrix.at<double>(1, 2) << " px\n";
    cout << "  畸变系数 (k1, k2, p1, p2, k3):\n";
    for (int i = 0; i < distortionCoeffs.rows; ++i) {
      cout << "    [" << i << "] = " << distortionCoeffs.at<double>(i, 0)
           << "\n";
    }
    cout << "\n";

    return true;

  } catch (const exception& e) {
    cerr << "[ERROR] 读取标定文件时出错: " << e.what() << endl;
    fs.release();
    return false;
  }
}

/**
 * @brief 从目录中读取所有支持的图像文件
 * @param imageDir 图像目录
 * @return 图像文件路径列表
 */
vector<string> readImagePaths(const string& imageDir) {
  vector<string> imagePaths;

  if (!fs::exists(imageDir)) {
    cerr << "[ERROR] 目录不存在: " << imageDir << endl;
    return imagePaths;
  }

  if (!fs::is_directory(imageDir)) {
    cerr << "[ERROR] 不是目录: " << imageDir << endl;
    return imagePaths;
  }

  // 遍历目录中的所有文件
  for (const auto& entry : fs::directory_iterator(imageDir)) {
    if (entry.is_regular_file()) {
      string filename = entry.path().filename().string();
      string extension = entry.path().extension().string();

      // 转换为小写进行比较
      transform(extension.begin(), extension.end(), extension.begin(),
                ::tolower);

      // 支持 BMP, JPG, PNG 格式
      if (extension == ".bmp" || extension == ".jpg" || extension == ".jpeg" ||
          extension == ".png") {
        imagePaths.push_back(entry.path().string());
      }
    }
  }

  // 按文件名排序
  sort(imagePaths.begin(), imagePaths.end());

  cout << "[INFO] 在目录中找到 " << imagePaths.size() << " 张图像\n";
  return imagePaths;
}

/**
 * @brief 对单张图像进行去畸变处理
 * @param image 输入图像
 * @param cameraMatrix 相机矩阵
 * @param distortionCoeffs 畸变系数
 * @return 去畸变后的图像
 */
Mat undistortImage(const Mat& image, const Mat& cameraMatrix,
                   const Mat& distortionCoeffs) {
  Mat undistorted;

  // 使用 cv::undistort 进行去畸变
  // 注意：这会使用相机矩阵和畸变系数进行反向映射
  cv::undistort(image, undistorted, cameraMatrix, distortionCoeffs);

  return undistorted;
}

/**
 * @brief 生成输出文件名
 * @param inputPath 输入图像路径
 * @param outputDir 输出目录
 * @return 输出文件路径
 */
string generateOutputPath(const string& inputPath, const string& outputDir) {
  fs::path inputPathObj(inputPath);
  string filename = inputPathObj.filename().string();
  string nameWithoutExt = inputPathObj.stem().string();
  string ext = inputPathObj.extension().string();

  // 生成输出文件名，添加 _undistorted 后缀
  string outputFilename = nameWithoutExt + "_undistorted" + ext;
  string outputPath = outputDir + "/" + outputFilename;

  return outputPath;
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

  string calibrationFile = argv[1];
  string inputDir = argv[2];
  string outputDir;

  // 如果没有指定输出目录，使用默认目录
  if (argc >= 4) {
    outputDir = argv[3];
  } else {
    outputDir = inputDir + "/undistorted";
  }

  // 解析可选参数
  for (int i = 3; i < argc; ++i) {
    string arg = argv[i];
    if (arg == "--help") {
      printUsage(argv[0]);
      return 0;
    }
  }

  cout << "\n";
  cout << "╔════════════════════════════════════════╗\n";
  cout << "║       相机图像去畸变程序 v1.0           ║\n";
  cout << "╚════════════════════════════════════════╝\n";

  cout << "\n[配置信息]\n";
  cout << "  标定文件: " << calibrationFile << "\n";
  cout << "  输入目录: " << inputDir << "\n";
  cout << "  输出目录: " << outputDir << "\n";
  cout << "\n";

  // ====================================================================
  // 步骤 1: 读取标定数据
  // ====================================================================
  Mat cameraMatrix, distortionCoeffs;
  if (!loadCalibrationData(calibrationFile, cameraMatrix, distortionCoeffs)) {
    cerr << "[ERROR] 读取标定数据失败\n";
    return -1;
  }

  // ====================================================================
  // 步骤 2: 读取图像列表
  // ====================================================================
  vector<string> imagePaths = readImagePaths(inputDir);
  if (imagePaths.empty()) {
    cerr << "[ERROR] 在目录中未找到任何图像文件\n";
    return -1;
  }

  // ====================================================================
  // 步骤 3: 创建输出目录
  // ====================================================================
  cout << "[步骤] 创建输出目录...\n";
  if (!fs::exists(outputDir)) {
    try {
      fs::create_directories(outputDir);
      cout << "[成功] 输出目录已创建: " << outputDir << "\n\n";
    } catch (const exception& e) {
      cerr << "[ERROR] 无法创建输出目录: " << e.what() << endl;
      return -1;
    }
  } else {
    cout << "[信息] 输出目录已存在: " << outputDir << "\n\n";
  }

  // ====================================================================
  // 步骤 4: 处理每张图像
  // ====================================================================
  cout << "[步骤] 处理图像...\n";
  cout << "========================================\n";

  int successCount = 0;
  int failureCount = 0;

  for (size_t i = 0; i < imagePaths.size(); ++i) {
    const string& imagePath = imagePaths[i];
    string filename = fs::path(imagePath).filename().string();

    cout << "[图像 " << (i + 1) << "/" << imagePaths.size() << "] " << filename;

    // 读取图像
    Mat image = imread(imagePath);
    if (image.empty()) {
      cout << " - [跳过] 无法读取文件\n";
      failureCount++;
      continue;
    }

    // 获取图像信息
    int imgWidth = image.cols;
    int imgHeight = image.rows;
    int channels = image.channels();

    if (DEBUG_UNDISTORT) {
      cout << " (分辨率: " << imgWidth << "x" << imgHeight
           << ", 通道: " << channels << ")";
    }

    // 进行去畸变处理
    Mat undistorted = undistortImage(image, cameraMatrix, distortionCoeffs);

    // 生成输出路径
    string outputPath = generateOutputPath(imagePath, outputDir);

    // 保存去畸变后的图像
    bool saveSuccess = imwrite(outputPath, undistorted);
    if (saveSuccess) {
      cout << " - [成功] 已保存\n";
      successCount++;
    } else {
      cout << " - [失败] 无法保存文件\n";
      failureCount++;
    }
  }

  cout << "========================================\n";
  cout << "[结果] 成功处理: " << successCount << "/" << imagePaths.size()
       << " 张图像\n";
  if (failureCount > 0) {
    cout << "[结果] 失败: " << failureCount << " 张图像\n";
  }

  // ====================================================================
  // 步骤 5: 显示总结信息
  // ====================================================================
  cout << "\n";
  cout << "╔════════════════════════════════════════╗\n";
  cout << "║       处理完成                          ║\n";
  cout << "╚════════════════════════════════════════╝\n";
  cout << "\n";

  if (successCount > 0) {
    cout << "[成功] " << successCount << " 张去畸变图像已保存到:\n";
    cout << "  " << outputDir << "\n";
    cout << "\n";
    cout << "[信息] 输出文件命名规则: <原始文件名>_undistorted.<扩展名>\n";
    cout << "  示例: image_001.bmp -> image_001_undistorted.bmp\n";
  } else {
    cout << "[警告] 没有成功处理任何图像\n";
  }

  cout << "\n";

  return (successCount > 0) ? 0 : -1;
}
