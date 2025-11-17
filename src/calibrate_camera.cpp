/**
 * @file calibrate_camera.cpp
 * @brief 相机标定程序 - 使用棋盘格标定板计算相机内参
 *
 * @details
 *   该程序实现了以下功能：
 *   1. 从指定目录读取标定图像（BMP格式）
 *   2. 检测棋盘格角点 (12x9 规格)
 *   3. 计算相机内参矩阵、畸变系数等
 *   4. 生成标定结果 YAML 文件
 *
 * 棋盘格规格：12 列 × 9 行黑白方格
 * 标定结果包含：
 *   - 相机矩阵 (Camera Matrix) - 3x3 内参矩阵
 *   - 畸变系数 (Distortion Coefficients) - 畸变参数
 *   - 重投影误差 (Reprojection Error) - 标定精度指标
 *
 * @usage
 *   ./calibrate_camera <image_directory> [--output result.yaml] [--square-size
 * 20]
 *
 * @example
 *   ./calibrate_camera /home/zzh/Pictures/hik
 *   ./calibrate_camera /home/zzh/Pictures/hik --output calib.yaml --square-size
 * 25
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

/// 棋盘格配置
static const int CHESSBOARD_COLS = 11;  ///< 棋盘格列数
static const int CHESSBOARD_ROWS = 8;   ///< 棋盘格行数
static const Size BOARD_SIZE(CHESSBOARD_COLS, CHESSBOARD_ROWS);

/// 棋盘格方块边长（毫米），根据实际标定板调整
static float SQUARE_SIZE_MM = 45.0f;

/// 检测参数
static const int CORNER_DETECTION_RETRIES = 5;  ///< 角点检测重试次数

/// 调试开关
static const bool DEBUG_FOCAL_LENGTH =
    false;  ///< 是否输出等效焦距的调试信息 (默认关闭，调试时设置为 true)

// ============================================================================
// 相机实际参数配置
// ============================================================================

/// 相机分辨率 (实际相机规格)
static const int CAMERA_WIDTH = 2448;   ///< 相机图像宽度（像素）
static const int CAMERA_HEIGHT = 2048;  ///< 相机图像高度（像素）

/// 相机镜头焦距 (实际相机规格)
static const float LENS_FOCAL_LENGTH_MM = 6.0f;  ///< 镜头标称焦距（毫米）

/// 基于相机传感器大小（Sony IMX264: 2/3", 像元 3.45μm × 3.45μm）
static const float SENSOR_WIDTH_MM = 8.4456f;   ///< 传感器宽度（毫米）
static const float SENSOR_HEIGHT_MM = 7.0656f;  ///< 传感器高度（毫米）

/// 基于传感器和焦距计算的理论焦距范围（像素）
/// Sony IMX264 规格:
///   - 像元尺寸: 3.45 μm × 3.45 μm
///   - 靶面尺寸: 2/3" (Sensor Width = 2448 × 3.45 / 1000 = 8.4456mm)
///   - 分辨率: 2448 × 2048
///
/// 计算: 焦距(px) = 镜头焦距(mm) * 图像宽度(px) / 传感器宽度(mm)
/// 理论值: 6.0 * 2448 / 8.4456 ≈ 1739.13 px
static const float EXPECTED_FOCAL_LENGTH_MIN = 1652.2f;  // 理论值 ±5%
static const float EXPECTED_FOCAL_LENGTH_MAX = 1826.1f;  // 理论值 ±5%
static const float EXPECTED_FOCAL_LENGTH_MIN_Y = 1652.2f;
static const float EXPECTED_FOCAL_LENGTH_MAX_Y = 1826.1f;

// ============================================================================
// 辅助结构体和函数
// ============================================================================

/// 标定结果结构体
struct CalibrationResult {
  Mat cameraMatrix;          ///< 相机内参矩阵 3x3
  Mat distortionCoeffs;      ///< 畸变系数（5个或8个）
  vector<Mat> rvecs;         ///< 旋转向量列表
  vector<Mat> tvecs;         ///< 平移向量列表
  double reprojectionError;  ///< 重投影平均误差
  int imageCount;            ///< 用于标定的图像数
  Size imageSize;            ///< 图像尺寸
  bool isValid;              ///< 标定结果是否有效
  string validationMessage;  ///< 校验信息
};

/// 校验结果结构体
struct ValidationResult {
  bool passed;              ///< 校验是否通过
  vector<string> warnings;  ///< 警告信息
  vector<string> errors;    ///< 错误信息
  string qualityRating;     ///< 质量评级
};

/**
 * @brief 打印程序用法
 */
void printUsage(const char* programName) {
  cout << "\n========================================\n";
  cout << "相机内参标定程序\n";
  cout << "========================================\n\n";
  cout << "用法:\n";
  cout << "  " << programName << " <image_directory> [options]\n\n";
  cout << "选项:\n";
  cout << "  --output <filename>    输出标定结果文件名 (默认: "
          "camera_calibration.yaml)\n";
  cout << "  --square-size <mm>     棋盘格方块边长，单位毫米 (默认: 20mm)\n";
  cout << "  --display              显示检测结果（仅调试用）\n";
  cout << "  --help                 显示此帮助信息\n\n";
  cout << "示例:\n";
  cout << "  " << programName << " /path/to/images\n";
  cout << "  " << programName
       << " /path/to/images --output calib.yaml --square-size 25\n";
  cout << "\n棋盘格规格: " << CHESSBOARD_COLS << " 列 × " << CHESSBOARD_ROWS
       << " 行\n";
  cout << "相机规格: " << CAMERA_WIDTH << " x " << CAMERA_HEIGHT << " 像素, "
       << LENS_FOCAL_LENGTH_MM << "mm 镜头\n";
  cout << "========================================\n\n";
}

/**
 * @brief 校验标定结果的合理性
 * @param result 标定结果
 * @return 校验结果
 *
 * 校验项：
 * 1. 图像数量是否充足 (建议 >= 20)
 * 2. 重投影误差是否在合理范围内 (< 2.0px)
 * 3. 焦距是否在合理范围内（与实际相机参数对比）
 * 4. 光心是否在图像范围内
 * 5. 畸变系数是否在合理范围内
 */
ValidationResult validateCalibrationResult(const CalibrationResult& result) {
  ValidationResult validation;
  validation.passed = true;
  validation.qualityRating = "⭐⭐⭐⭐⭐ 优秀";

  double fx = result.cameraMatrix.at<double>(0, 0);
  double fy = result.cameraMatrix.at<double>(1, 1);
  double cx = result.cameraMatrix.at<double>(0, 2);
  double cy = result.cameraMatrix.at<double>(1, 2);

  // ====================================================================
  // 检验 1: 图像数量
  // ====================================================================
  if (result.imageCount < 3) {
    validation.errors.push_back(
        "❌ 图像数量过少 (检测到: " + to_string(result.imageCount) +
        ", 最小需要: 3)");
    validation.passed = false;
  } else if (result.imageCount < 10) {
    validation.warnings.push_back(
        "⚠️  图像数量偏少 (检测到: " + to_string(result.imageCount) +
        ", 推荐: >= 20)");
  } else if (result.imageCount < 20) {
    validation.warnings.push_back(
        "⚠️  图像数量较少 (检测到: " + to_string(result.imageCount) +
        ", 推荐: >= 20)");
  }

  // ====================================================================
  // 检验 2: 重投影误差 (RMS Error)
  // ====================================================================
  if (result.reprojectionError > 2.0) {
    validation.errors.push_back("❌ 重投影误差过大 (" +
                                to_string(result.reprojectionError) +
                                "px > 2.0px)，标定质量不可靠");
    validation.passed = false;
    validation.qualityRating = "⭐⭐ 需要改进";
  } else if (result.reprojectionError > 1.0) {
    validation.warnings.push_back("⚠️  重投影误差 (" +
                                  to_string(result.reprojectionError) +
                                  "px)，可接受但仍需改进");
    validation.qualityRating = "⭐⭐⭐ 可接受";
  } else if (result.reprojectionError > 0.5) {
    validation.qualityRating = "⭐⭐⭐⭐ 良好";
  }

  // ====================================================================
  // 检验 3: 焦距合理性 (与实际相机参数对比)
  // ====================================================================
  // 相机规格: Sony IMX264
  //   - 分辨率: 2448×2048 px
  //   - 像元尺寸: 3.45μm × 3.45μm
  //   - 传感器宽度: 8.4456mm
  //   - 镜头焦距: 6mm
  // 理论焦距: f(px) = 6.0 * 2448 / 8.4456 ≈ 1739.13 px
  //
  // 校验策略: 采用两层校验
  //   1. 严格范围 (±5%): 检测标定异常
  //   2. 广泛范围 (0.8-1.5×分辨率): 确保数据可用

  // 严格范围校验: ±5% 的理论值
  bool fxInStrict =
      (fx >= EXPECTED_FOCAL_LENGTH_MIN && fx <= EXPECTED_FOCAL_LENGTH_MAX);
  bool fyInStrict =
      (fy >= EXPECTED_FOCAL_LENGTH_MIN_Y && fy <= EXPECTED_FOCAL_LENGTH_MAX_Y);

  // 广泛范围校验: 0.8-1.5 倍分辨率
  double minBroadFx = CAMERA_WIDTH * 0.8;   // ~1958
  double maxBroadFx = CAMERA_WIDTH * 1.5;   // ~3672
  double minBroadFy = CAMERA_HEIGHT * 0.8;  // ~1638
  double maxBroadFy = CAMERA_HEIGHT * 1.5;  // ~3072

  if (!fxInStrict) {
    if (fx < minBroadFx || fx > maxBroadFx) {
      validation.warnings.push_back(
          "⚠️  焦距 fx (" + to_string(fx) + "px) 严重超出理论范围 [" +
          to_string(EXPECTED_FOCAL_LENGTH_MIN) + ", " +
          to_string(EXPECTED_FOCAL_LENGTH_MAX) + "]");
    } else {
      validation.warnings.push_back(
          "⚠️  焦距 fx (" + to_string(fx) + "px) 偏离理论值，期望范围 [" +
          to_string(EXPECTED_FOCAL_LENGTH_MIN) + ", " +
          to_string(EXPECTED_FOCAL_LENGTH_MAX) + "]");
    }
  }

  if (!fyInStrict) {
    if (fy < minBroadFy || fy > maxBroadFy) {
      validation.warnings.push_back(
          "⚠️  焦距 fy (" + to_string(fy) + "px) 严重超出理论范围 [" +
          to_string(EXPECTED_FOCAL_LENGTH_MIN_Y) + ", " +
          to_string(EXPECTED_FOCAL_LENGTH_MAX_Y) + "]");
    } else {
      validation.warnings.push_back(
          "⚠️  焦距 fy (" + to_string(fy) + "px) 偏离理论值，期望范围 [" +
          to_string(EXPECTED_FOCAL_LENGTH_MIN_Y) + ", " +
          to_string(EXPECTED_FOCAL_LENGTH_MAX_Y) + "]");
    }
  }

  // 计算等效焦距（毫米）用于参考
  double equivalentFocalLength = fx * SENSOR_WIDTH_MM / CAMERA_WIDTH;

  if (DEBUG_FOCAL_LENGTH) {
    cout << "\n[校验] 焦距分析 (Sony IMX264):\n";
    cout << "  传感器规格: " << CAMERA_WIDTH << "×" << CAMERA_HEIGHT
         << " px, 像元 3.45μm, 2/3\" 靶面\n";
    cout << "  镜头标称焦距: " << LENS_FOCAL_LENGTH_MM << " mm\n";
    cout << "  传感器宽度: " << fixed << setprecision(4) << SENSOR_WIDTH_MM
         << " mm\n";
    cout << "  理论焦距: 6.0 × " << CAMERA_WIDTH << " / " << SENSOR_WIDTH_MM
         << " = " << fixed << setprecision(2)
         << (LENS_FOCAL_LENGTH_MM * CAMERA_WIDTH / SENSOR_WIDTH_MM) << " px\n";
    cout << "  标定焦距(等效): " << fixed << setprecision(2)
         << equivalentFocalLength << " mm\n";
    cout << "  焦距偏差: " << abs(equivalentFocalLength - LENS_FOCAL_LENGTH_MM)
         << " mm";

    // 评估焦距差异
    double focalDiff = abs(equivalentFocalLength - LENS_FOCAL_LENGTH_MM);
    if (focalDiff < 0.3) {
      cout << " (优秀: < 0.3mm) ✅\n";
    } else if (focalDiff < 0.8) {
      cout << " (良好: < 0.8mm) ✅\n";
    } else if (focalDiff < 1.5) {
      cout << " (可接受: < 1.5mm) ⚠️\n";
      if (!validation.warnings.empty()) {
        // 只在有其他警告时添加
      } else {
        validation.warnings.push_back("⚠️  焦距与标称值偏差 " +
                                      to_string(focalDiff) +
                                      "mm，可能需要重新标定");
      }
    } else {
      cout << " (需要检查) ❌\n";
      validation.warnings.push_back("❌ 焦距与标称值偏差 " +
                                    to_string(focalDiff) +
                                    "mm，建议重新采集或调整参数");
    }
  }

  // ====================================================================
  // 检验 4: 光心位置 (应在图像中心附近)
  // ====================================================================
  double expectedCx = CAMERA_WIDTH / 2.0;
  double expectedCy = CAMERA_HEIGHT / 2.0;
  double cxMarginMin = CAMERA_WIDTH * 0.1;  // 光心应在宽度的 10%-90%
  double cxMarginMax = CAMERA_WIDTH * 0.9;
  double cyMarginMin = CAMERA_HEIGHT * 0.1;  // 光心应在高度的 10%-90%
  double cyMarginMax = CAMERA_HEIGHT * 0.9;

  if (cx < cxMarginMin || cx > cxMarginMax) {
    validation.errors.push_back(
        "❌ 光心 cx (" + to_string(cx) + "px) 位置异常，应在 [" +
        to_string(cxMarginMin) + ", " + to_string(cxMarginMax) + "] 范围内");
    validation.passed = false;
  }

  if (cy < cyMarginMin || cy > cyMarginMax) {
    validation.errors.push_back(
        "❌ 光心 cy (" + to_string(cy) + "px) 位置异常，应在 [" +
        to_string(cyMarginMin) + ", " + to_string(cyMarginMax) + "] 范围内");
    validation.passed = false;
  }

  // 提示光心偏离中心
  double cxDeviation = abs(cx - expectedCx);
  double cyDeviation = abs(cy - expectedCy);
  if (cxDeviation > CAMERA_WIDTH * 0.1 || cyDeviation > CAMERA_HEIGHT * 0.1) {
    validation.warnings.push_back(
        "⚠️  光心(" + to_string(cx) + ", " + to_string(cy) + ")偏离图像中心(" +
        to_string(expectedCx) + ", " + to_string(expectedCy) + ")较远");
  }

  // ====================================================================
  // 检验 5: 畸变系数合理性
  // ====================================================================
  double k1 = result.distortionCoeffs.at<double>(0, 0);
  double k2 = result.distortionCoeffs.at<double>(1, 0);
  double p1 = result.distortionCoeffs.at<double>(2, 0);
  double p2 = result.distortionCoeffs.at<double>(3, 0);
  double k3 = result.distortionCoeffs.at<double>(4, 0);

  // 畸变系数通常在 [-1, 1] 范围内
  if (abs(k1) > 1.0 || abs(k2) > 1.0 || abs(k3) > 1.0) {
    validation.warnings.push_back("⚠️  径向畸变系数 (k1=" + to_string(k1) +
                                  ", k2=" + to_string(k2) +
                                  ", k3=" + to_string(k3) + ") 可能异常");
  }

  if (abs(p1) > 0.1 || abs(p2) > 0.1) {
    validation.warnings.push_back("⚠️  切向畸变系数 (p1=" + to_string(p1) +
                                  ", p2=" + to_string(p2) + ") 较大");
  }

  // ====================================================================
  // 综合评估
  // ====================================================================
  if (!validation.errors.empty()) {
    validation.passed = false;
    if (validation.qualityRating == "⭐⭐⭐⭐⭐ 优秀") {
      validation.qualityRating = "⭐⭐ 需要改进";
    }
  }

  return validation;
}

/**
 * @brief 从目录中读取所有 BMP 图像
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
 * @brief 检测单张图像中的棋盘格角点
 * @param image 输入图像
 * @param corners 输出角点 (2D)
 * @param displayDebug 是否显示调试信息
 * @return 是否检测成功
 */
bool detectChessboardCorners(const Mat& image, vector<Point2f>& corners,
                             bool displayDebug = false) {
  Mat gray;
  static int imageCounter = 0;  // 用于保存调试图像
  imageCounter++;

  // 转换为灰度图
  if (image.channels() == 3) {
    cvtColor(image, gray, COLOR_BGR2GRAY);
    std::cout << "  [信息] 输入图像为彩色，已转换为灰度图。\n";
  } else {
    gray = image.clone();
    std::cout << "  [信息] 输入图像为灰度图。\n";
  }

  // 保存灰度图用于调试
  if (displayDebug) {
    string grayFileName = "debug_gray_" + to_string(imageCounter) + ".png";
    imwrite(grayFileName, gray);
    cout << "  [调试] 灰度图已保存: " << grayFileName << "\n";
  }

  // 尝试多种检测方法
  bool found = false;
  int attemptNum = 0;

  // 方法 1: ADAPTIVE_THRESH + NORMALIZE
  attemptNum++;
  int flags1 = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  found = findChessboardCorners(gray, BOARD_SIZE, corners, flags1);
  cout << "  [尝试 " << attemptNum
       << "] ADAPTIVE_THRESH + NORMALIZE: " << (found ? "✓ 成功" : "✗ 失败")
       << " (检测到 " << corners.size() << " 个角点)\n";

  // 方法 4: 无选项 (默认)
  if (!found) {
    attemptNum++;
    corners.clear();
    found = findChessboardCorners(gray, BOARD_SIZE, corners, 0);
    cout << "  [尝试 " << attemptNum
         << "] 默认选项 (无flags): " << (found ? "✓ 成功" : "✗ 失败")
         << " (检测到 " << corners.size() << " 个角点)\n";
  }

  // 图像分析信息
  if (displayDebug || !found) {
    cout << "  [图像分析]\n";
    cout << "    - 图像尺寸: " << gray.cols << " x " << gray.rows << "\n";

    // 改进的数据类型输出
    int type = gray.type();
    string typeStr;
    if (type == CV_8UC1) {
      typeStr = "CV_8UC1 (8位无符号单通道 - 标准灰度图) ✓";
    } else if (type == CV_8UC3) {
      typeStr = "CV_8UC3 (8位无符号三通道 - BGR彩色图)";
    } else if (type == CV_32F) {
      typeStr = "CV_32F (32位浮点)";
    } else {
      typeStr = "未知类型 ⚠️";
    }
    cout << "    - 数据类型: " << type << " (" << typeStr << ")\n";

    cout << "    - 像素值范围: ";
    double minVal, maxVal;
    cv::minMaxLoc(gray, &minVal, &maxVal);
    cout << (int)minVal << " - " << (int)maxVal << "\n";
    cout << "    - 平均像素值: " << (int)cv::mean(gray).val[0] << "\n";
    cout << "    - 对比度 (标准差): ";
    Mat mean, stddev;
    cv::meanStdDev(gray, mean, stddev);
    cout << fixed << setprecision(2) << stddev.at<double>(0, 0) << "\n";

    // 分析对比度是否足够
    double contrast = stddev.at<double>(0, 0);
    if (contrast < 10) {
      cout << "      ⚠️ 图像对比度很低（<10），可能导致检测失败\n";
    } else if (contrast < 30) {
      cout << "      ⚠️ 图像对比度较低（<30），检测可能困难\n";
    } else {
      cout << "      ✓ 图像对比度良好（>=30）\n";
    }

    // 检测棋盘格的边缘信息
    Mat edges;
    Canny(gray, edges, 50, 150);
    int edgePixels = cv::countNonZero(edges);
    cout << "    - 边缘像素数: " << edgePixels << " (" << fixed
         << setprecision(2) << (100.0 * edgePixels / (gray.rows * gray.cols))
         << "% 的图像)\n";
    if (edgePixels < gray.rows * gray.cols * 0.01) {
      cout << "      ⚠️ 边缘信息极少，棋盘格可能很模糊或不存在\n";
    }

    cout << "    - 棋盘格规格: " << BOARD_SIZE.width << " x "
         << BOARD_SIZE.height << " (共 " << BOARD_SIZE.width * BOARD_SIZE.height
         << " 个角点)\n";
  }

  if (found) {
    // 细化角点位置
    cornerSubPix(
        gray, corners, Size(11, 11), Size(-1, -1),
        TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001));

    cout << "  [角点细化] 完成，精化后角点数: " << corners.size() << "\n";

    // 保存检测结果图像
    if (displayDebug) {
      Mat display = image.clone();
      drawChessboardCorners(display, BOARD_SIZE, corners, found);
      string resultFileName =
          "debug_detected_" + to_string(imageCounter) + ".png";
      imwrite(resultFileName, display);
      cout << "  [调试] 检测结果图已保存: " << resultFileName << "\n";
      imshow("检测结果", display);
      waitKey(1);
    }

    // 打印前几个角点的坐标
    cout << "  [角点坐标] 前 3 个角点:\n";
    for (int i = 0; i < min(3, (int)corners.size()); i++) {
      cout << "    角点 " << i << ": (" << fixed << setprecision(2)
           << corners[i].x << ", " << corners[i].y << ")\n";
    }
  } else {
    cout << "  [警告] 所有检测方法均失败，无法找到棋盘格！\n";
    cout << "  [建议]\n";
    cout << "    1. 检查图像是否真的包含棋盘格\n";
    cout << "    2. 检查棋盘格规格是否正确 (期望: " << BOARD_SIZE.width << " x "
         << BOARD_SIZE.height << ")\n";
    cout << "    3. 检查棋盘格是否清晰、无遮挡\n";
    cout << "    4. 检查光照是否充足\n";
    cout << "    5. 使用 --display 选项查看保存的调试图像\n";

    // 保存失败的原始图像用于后续分析
    if (displayDebug) {
      string failFileName = "debug_failed_" + to_string(imageCounter) + ".png";
      imwrite(failFileName, image);
      cout << "  [调试] 原始图像已保存: " << failFileName << "\n";
    }
  }

  return found;
}

/**
 * @brief 从标定图像集合中提取棋盘格角点
 * @param imagePaths 图像文件路径列表
 * @param objectPoints 输出的3D对象点列表
 * @param imagePoints 输出的2D图像点列表
 * @param imageSize 图像尺寸 (输出参数)
 * @param displayDebug 是否显示调试信息
 * @return 成功检测的图像数量
 */
int extractCornerPoints(const vector<string>& imagePaths,
                        vector<vector<Point3f>>& objectPoints,
                        vector<vector<Point2f>>& imagePoints, Size& imageSize,
                        bool displayDebug = false) {
  // 生成标准棋盘格的3D点坐标
  // 棋盘格的第一个角点在 (0,0,0)，方向为 XY 平面
  vector<Point3f> objp;
  for (int i = 0; i < CHESSBOARD_ROWS; ++i) {
    for (int j = 0; j < CHESSBOARD_COLS; ++j) {
      objp.push_back(Point3f(j * SQUARE_SIZE_MM, i * SQUARE_SIZE_MM, 0.0f));
    }
  }

  int successCount = 0;

  cout << "\n[步骤] 检测棋盘格角点...\n";
  cout << "========================================\n";

  for (size_t i = 0; i < imagePaths.size(); ++i) {
    const string& imagePath = imagePaths[i];
    cout << "[图像 " << (i + 1) << "/" << imagePaths.size() << "] ";
    cout << fs::path(imagePath).filename().string();

    // 读取图像
    Mat image = imread(imagePath);
    if (image.empty()) {
      cout << " - [跳过] 无法读取文件\n";
      continue;
    }

    // 保存图像尺寸（所有图像应该相同）
    if (imageSize.empty()) {
      imageSize = image.size();
      cout << " (尺寸: " << imageSize.width << " x " << imageSize.height
           << ", 通道: " << image.channels() << ")\n";
    } else {
      // 检查后续图像是否与第一张图像尺寸一致
      if (image.size() != imageSize) {
        cout << " - [警告] 图像尺寸不一致: " << image.size().width << " x "
             << image.size().height << " (期望: " << imageSize.width << " x "
             << imageSize.height << ")\n";
      }
    }

    // 检测角点
    vector<Point2f> corners;
    if (detectChessboardCorners(image, corners, displayDebug)) {
      objectPoints.push_back(objp);
      imagePoints.push_back(corners);
      successCount++;
      cout << " - [成功] 检测到 " << corners.size() << " 个角点\n";
    } else {
      cout << " - [失败] 无法检测到棋盘格\n";
    }
  }

  cout << "========================================\n";
  cout << "[结果] 成功检测: " << successCount << "/" << imagePaths.size()
       << " 张图像\n\n";

  return successCount;
}

/**
 * @brief 执行相机标定计算
 * @param objectPoints 3D对象点列表
 * @param imagePoints 2D图像点列表
 * @param imageSize 图像尺寸
 * @return 标定结果
 *
 * 注: 初始化时使用实际相机参数作为初值，提高收敛精度
 */
CalibrationResult calibrateCamera(const vector<vector<Point3f>>& objectPoints,
                                  const vector<vector<Point2f>>& imagePoints,
                                  const Size& imageSize) {
  CalibrationResult result;
  result.imageSize = imageSize;
  result.imageCount = objectPoints.size();

  cout << "\n[步骤] 计算相机内参...\n";
  cout << "========================================\n";

  // 初始化相机矩阵 - 使用实际相机参数
  // 方案: 对于标准工业相机，光心通常在或接近图像中心
  result.cameraMatrix = Mat::eye(3, 3, CV_64F);

  // 焦距初值: 使用实际镜头参数计算的理论值
  // f(px) = 镜头焦距(mm) × 图像宽度(px) / 传感器宽度(mm)
  double theoreticalFx =
      LENS_FOCAL_LENGTH_MM * imageSize.width / SENSOR_WIDTH_MM;
  double theoreticalFy =
      LENS_FOCAL_LENGTH_MM * imageSize.height / SENSOR_WIDTH_MM;

  // 如果是标准分辨率，使用理论值；否则使用图像尺寸作为启发式初值
  if (imageSize.width == CAMERA_WIDTH && imageSize.height == CAMERA_HEIGHT) {
    result.cameraMatrix.at<double>(0, 0) = theoreticalFx;  // fx
    result.cameraMatrix.at<double>(1, 1) = theoreticalFy;  // fy
  } else {
    // 降级方案：使用图像尺寸
    result.cameraMatrix.at<double>(0, 0) = imageSize.width;   // fx
    result.cameraMatrix.at<double>(1, 1) = imageSize.height;  // fy
  }

  // 光心: 通常在图像中心
  result.cameraMatrix.at<double>(0, 2) = imageSize.width / 2.0;   // cx
  result.cameraMatrix.at<double>(1, 2) = imageSize.height / 2.0;  // cy

  cout << "[信息] 初始化参数:\n";
  cout << "  图像尺寸: " << imageSize.width << " x " << imageSize.height
       << " px\n";
  cout << "  相机规格: Sony IMX264 (2448×2048 px)\n";
  cout << "  像元尺寸: 3.45μm × 3.45μm\n";
  cout << "  靶面尺寸: 2/3\" (传感器宽度 " << fixed << setprecision(4)
       << SENSOR_WIDTH_MM << " mm)\n";
  cout << "  镜头焦距: " << LENS_FOCAL_LENGTH_MM << " mm\n";
  cout << "  理论焦距: fx=" << fixed << setprecision(2) << theoreticalFx
       << "px, fy=" << theoreticalFy << "px\n";
  cout << "  初始光心: (" << imageSize.width / 2.0 << ", "
       << imageSize.height / 2.0 << ")\n\n";

  // 执行标定
  // 使用 5 个畸变系数 (k1, k2, p1, p2, k3)
  result.distortionCoeffs = Mat::zeros(5, 1, CV_64F);

  int calibFlags = 0;  // 0 表示所有参数都自由优化

  double rmsError = calibrateCamera(
      objectPoints, imagePoints, imageSize, result.cameraMatrix,
      result.distortionCoeffs, result.rvecs, result.tvecs, calibFlags);

  result.reprojectionError = rmsError;

  cout << "[完成] 标定完毕\n";
  cout << "========================================\n\n";

  return result;
}

/**
 * @brief 打印标定结果
 */
void printCalibrationResult(const CalibrationResult& result) {
  cout << "\n========================================\n";
  cout << "标定结果\n";
  cout << "========================================\n\n";

  cout << "图像尺寸: " << result.imageSize.width << " x "
       << result.imageSize.height << " 像素\n";
  cout << "使用的标定图像数: " << result.imageCount << "\n";
  cout << "棋盘格规格: " << CHESSBOARD_COLS << " x " << CHESSBOARD_ROWS
       << " (列 x 行)\n";
  cout << "方块边长: " << SQUARE_SIZE_MM << " mm\n\n";

  cout << "────── 相机矩阵 (Camera Matrix) ──────\n";
  cout << "单位: 像素\n";
  cout << "  fx = " << result.cameraMatrix.at<double>(0, 0) << " px\n";
  cout << "  fy = " << result.cameraMatrix.at<double>(1, 1) << " px\n";
  cout << "  cx = " << result.cameraMatrix.at<double>(0, 2) << " px\n";
  cout << "  cy = " << result.cameraMatrix.at<double>(1, 2) << " px\n\n";

  cout << "相机矩阵 K:\n";
  cout << result.cameraMatrix << "\n\n";

  cout << "────── 畸变系数 (Distortion Coefficients) ──────\n";
  cout << "使用的模型: 5参数 (k1, k2, p1, p2, k3)\n";
  cout << "  k1 (径向畸变1): " << result.distortionCoeffs.at<double>(0, 0)
       << "\n";
  cout << "  k2 (径向畸变2): " << result.distortionCoeffs.at<double>(1, 0)
       << "\n";
  cout << "  p1 (切向畸变1): " << result.distortionCoeffs.at<double>(2, 0)
       << "\n";
  cout << "  p2 (切向畸变2): " << result.distortionCoeffs.at<double>(3, 0)
       << "\n";
  cout << "  k3 (径向畸变3): " << result.distortionCoeffs.at<double>(4, 0)
       << "\n\n";

  cout << "────── 标定精度 ──────\n";
  cout << "重投影误差 (RMS): " << result.reprojectionError << " px\n";
  cout << "  说明: 误差越小说明标定精度越高，通常 < 1.0px 为优，< 0.5px "
          "为优秀\n\n";

  cout << "========================================\n\n";
}

/**
 * @brief 打印标定校验结果
 */
void printValidationResult(const ValidationResult& validation) {
  cout << "\n╔════════════════════════════════════════╗\n";
  cout << "║       标定结果校验                      ║\n";
  cout << "╚════════════════════════════════════════╝\n\n";

  // 打印错误信息
  if (!validation.errors.empty()) {
    cout << "【错误】:\n";
    for (const auto& error : validation.errors) {
      cout << "  " << error << "\n";
    }
    cout << "\n";
  }

  // 打印警告信息
  if (!validation.warnings.empty()) {
    cout << "【警告】:\n";
    for (const auto& warning : validation.warnings) {
      cout << "  " << warning << "\n";
    }
    cout << "\n";
  }

  // 打印总体评估
  cout << "【评估结果】\n";
  cout << "  质量评级: " << validation.qualityRating << "\n";

  if (validation.passed) {
    cout << "  状态: ✅ 标定结果良好，可以使用\n";
  } else {
    cout << "  状态: ❌ 标定结果有问题，建议重新采集或调整参数\n";
  }

  cout << "\n========================================\n\n";
}

/**
 * @brief 将标定结果保存为 YAML 文件
 * @param filename 输出文件名
 * @param result 标定结果
 */
void saveCalibrationToYAML(const string& filename,
                           const CalibrationResult& result) {
  cout << "[步骤] 保存标定结果到文件: " << filename << "\n";

  FileStorage fs(filename, FileStorage::WRITE);

  if (!fs.isOpened()) {
    cerr << "[ERROR] 无法打开文件进行写入: " << filename << endl;
    return;
  }

  // 写入基本信息
  fs << "image_width" << result.imageSize.width;
  fs << "image_height" << result.imageSize.height;
  fs << "image_count" << result.imageCount;
  fs << "chessboard_cols" << CHESSBOARD_COLS;
  fs << "chessboard_rows" << CHESSBOARD_ROWS;
  fs << "square_size_mm" << SQUARE_SIZE_MM;
  fs << "reprojection_error" << result.reprojectionError;

  // 写入相机矩阵
  fs << "camera_matrix" << result.cameraMatrix;

  // 写入畸变系数
  fs << "distortion_coefficients" << result.distortionCoeffs;

  fs.release();

  cout << "[成功] 标定结果已保存\n\n";
}

/**
 * @brief 将标定结果保存为易读的文本文件
 * @param filename 输出文件名
 * @param result 标定结果
 */
void saveCalibrationToText(const string& filename,
                           const CalibrationResult& result) {
  string textFile = filename;
  size_t dotPos = textFile.rfind('.');
  if (dotPos != string::npos) {
    textFile = textFile.substr(0, dotPos);
  }
  textFile += ".txt";

  ofstream file(textFile);
  if (!file.is_open()) {
    cerr << "[ERROR] 无法打开文件: " << textFile << endl;
    return;
  }

  file << "相机标定结果\n";
  file << "================================\n\n";

  file << "图像分辨率: " << result.imageSize.width << " x "
       << result.imageSize.height << "\n";
  file << "使用图像数: " << result.imageCount << "\n";
  file << "棋盘格规格: " << CHESSBOARD_COLS << " x " << CHESSBOARD_ROWS << "\n";
  file << "方块边长: " << SQUARE_SIZE_MM << " mm\n";
  file << "重投影误差: " << result.reprojectionError << " px\n\n";

  file << "相机矩阵 (3x3):\n";
  file << result.cameraMatrix << "\n\n";

  file << "焦距:\n";
  file << "  fx = " << result.cameraMatrix.at<double>(0, 0) << " px\n";
  file << "  fy = " << result.cameraMatrix.at<double>(1, 1) << " px\n\n";

  file << "主点(光心):\n";
  file << "  cx = " << result.cameraMatrix.at<double>(0, 2) << " px\n";
  file << "  cy = " << result.cameraMatrix.at<double>(1, 2) << " px\n\n";

  file << "畸变系数 (5参数 k1, k2, p1, p2, k3):\n";
  for (int i = 0; i < result.distortionCoeffs.rows; ++i) {
    file << "  [" << i << "] = " << result.distortionCoeffs.at<double>(i, 0)
         << "\n";
  }
  file << "\n";

  file << "================================\n";
  file << "标定完成\n";

  file.close();
  cout << "[成功] 文本结果已保存到: " << textFile << "\n\n";
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
  // 解析命令行参数
  if (argc < 2) {
    printUsage(argv[0]);
    return -1;
  }

  string imageDir = argv[1];
  string outputFile = "camera_calibration.yaml";
  bool displayDebug = false;

  // 解析可选参数
  for (int i = 2; i < argc; ++i) {
    string arg = argv[i];

    if (arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--output" && i + 1 < argc) {
      outputFile = argv[++i];
    } else if (arg == "--square-size" && i + 1 < argc) {
      SQUARE_SIZE_MM = stof(argv[++i]);
    } else if (arg == "--display") {
      displayDebug = true;
    }
  }

  cout << "\n";
  cout << "╔════════════════════════════════════════╗\n";
  cout << "║     相机内参标定程序 v1.1              ║\n";
  cout << "║     12×9 棋盘格标定                    ║\n";
  cout << "║     相机规格: 2448×2048, 6mm 镜头     ║\n";
  cout << "╚════════════════════════════════════════╝\n";

  cout << "\n[配置信息]\n";
  cout << "  图像目录: " << imageDir << "\n";
  cout << "  输出文件: " << outputFile << "\n";
  cout << "  棋盘格大小: " << CHESSBOARD_COLS << " x " << CHESSBOARD_ROWS
       << "\n";
  cout << "  方块边长: " << SQUARE_SIZE_MM << " mm\n";
  cout << "  相机参数: " << CAMERA_WIDTH << "×" << CAMERA_HEIGHT << " px, "
       << LENS_FOCAL_LENGTH_MM << "mm 镜头\n";
  cout << "\n";

  // ====================================================================
  // 步骤 1: 读取图像
  // ====================================================================
  vector<string> imagePaths = readImagePaths(imageDir);
  if (imagePaths.empty()) {
    cerr << "[ERROR] 在目录中未找到任何图像文件\n";
    return -1;
  }

  // ====================================================================
  // 步骤 2: 检测棋盘格角点
  // ====================================================================
  vector<vector<Point3f>> objectPoints;  // 3D 点
  vector<vector<Point2f>> imagePoints;   // 2D 点
  Size imageSize;

  int successCount = extractCornerPoints(imagePaths, objectPoints, imagePoints,
                                         imageSize, displayDebug);

  if (successCount < 3) {
    cerr << "[ERROR] 成功检测的图像过少 (需要至少 3 张)\n";
    cerr << "        请检查:\n";
    cerr << "        1. 棋盘格是否正确 (12x9)\n";
    cerr << "        2. 图像质量是否足够\n";
    cerr << "        3. 棋盘格是否清晰可见\n";
    return -1;
  }

  // ====================================================================
  // 步骤 3: 标定计算
  // ====================================================================
  CalibrationResult result =
      calibrateCamera(objectPoints, imagePoints, imageSize);

  // ====================================================================
  // 步骤 4: 显示结果
  // ====================================================================
  printCalibrationResult(result);

  // ====================================================================
  // 步骤 4.5: 校验结果
  // ====================================================================
  ValidationResult validation = validateCalibrationResult(result);
  printValidationResult(validation);

  // ====================================================================
  // 步骤 5: 保存结果
  // ====================================================================
  saveCalibrationToYAML(outputFile, result);
  saveCalibrationToText(outputFile, result);

  cout << "[完成] 标定程序执行完毕\n";
  cout << "\n";

  if (displayDebug) {
    cout << "按任意键退出...\n";
    waitKey(0);
    destroyAllWindows();
  }

  return 0;
}
