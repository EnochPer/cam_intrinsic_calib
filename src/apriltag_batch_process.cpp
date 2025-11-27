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
 * <tag_size> [z_constraint] [--draw]
 *
 * @example
   ./apriltag_batch_process images/ camera_calib.yaml extrinsic.yaml 0.1
   ./apriltag_batch_process /home/zzh/hikon_cam/picture_data/20251121_174435/ \
 /home/zzh/hikon_cam/camera_calib.yaml /home/zzh/hikon_cam/extrinsic1.yaml \
 0.1 0.215 --ba
   ./apriltag_batch_process images/ camera_calib.yaml extrinsic.yaml 0.1 --draw
 */

#include "apriltag_batch_process.h"

#include "BA_tag_window.cpp"  // 包含 Bundle Adjustment 实现

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
  cout << "  --draw                 在保存的图像中绘制 AprilTag 角点和中心\n";
  cout << "  --method1              使用位姿估计方法1: estimate_tag_pose()\n";
  cout << "  --method2 或 --pnp     使用位姿估计方法2: PnP\n";
  cout << "  --method3 或 --ba      使用位姿估计方法3: Bundle Adjustment\n";
  cout << "  --method4 或 --ray     使用位姿估计方法4: Ray-Plane Intersection "
          "(强制约束Z高度)\n";
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
AprilTagDetection detectAprilTags(const Mat& image, double tagSize,
                                  const Mat& cameraMatrix,
                                  const Mat& distCoeffs) {
  AprilTagDetection detection;

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

  // 用于位姿估计的相机内参
  apriltag_detection_info_t info;
  info.det = nullptr;
  info.tagsize = tagSize;
  info.fx = cameraMatrix.at<double>(0, 0);
  info.fy = cameraMatrix.at<double>(1, 1);
  info.cx = cameraMatrix.at<double>(0, 2);
  info.cy = cameraMatrix.at<double>(1, 2);

  apriltag_detection_t* det = nullptr;
  zarray_get(detections_array, 0, &det);
  // AprilTagDetection detection;
  detection.tagId = det->id;

  // 提取四个角点和中心
  for (int j = 0; j < 4; ++j) {
    detection.corners.push_back(Point2f(det->p[j][0], det->p[j][1]));
  }
  detection.center = Point2f(det->c[0], det->c[1]);

  // 亚像素角点精化（提高像素级精度）
  TermCriteria subpixelCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30,
                                0.01);
  cornerSubPix(gray, detection.corners, Size(11, 11), Size(-1, -1),
               subpixelCriteria);

  // 方法1: 保持原有的estimate_tag_pose()
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
    detection.translationVector.at<double>(r, 0) =
        MATD_EL(pose.t, r, 0) * 1000;  // 转换为 mm
  }

  Rodrigues(detection.rotationMatrix, detection.rotationVector);

  cout << "  [DEBUG] 单应矩阵estimate_tag_pose位姿估计结果,相机坐标系：" << endl;
  // cout << "    旋转向量：" << detection.rotationVector << endl;
  cout << "    平移向量：" << detection.translationVector << endl;

  detection.isValid = true;

  // 清理位姿结构体
  matd_destroy(pose.R);
  matd_destroy(pose.t);

  // 清理资源
  image_u8_destroy(img_u8);
  apriltag_detections_destroy(detections_array);
  apriltag_detector_destroy(td);
  tagStandard41h12_destroy(tf);

  return detection;
}

/**
 * @brief 将相机坐标系的位姿变换到世界坐标系
 * @param usePnP 是否使用PnP结果进行位姿变换
 * @param useBA 是否使用Bundle Adjustment结果进行位姿变换
 */
AprilTagWorldPose transformToWorldCoordinates(
    const AprilTagDetection& cameraDetection,
    const CameraExtrinsics& cameraExtrinsics,
    bool useBA) {  // 默认使用原始的estimate_tag_pose结果
  AprilTagWorldPose worldPose;
  worldPose.tagId = cameraDetection.tagId;

  // 获取世界到相机的变换（外参）,世界坐标系原点在相机坐标系中的位姿
  Mat R_c2w = cameraExtrinsics.rotationMatrix;
  Mat t_c2w = cameraExtrinsics.translationVector;
  // 获取相机到世界的变换
  Mat R_w2c = R_c2w.t();
  Mat t_w2c = -R_c2w * t_c2w;
  // 获取相机到标签的变换，即标签在相机坐标系中的位姿
  Mat R_c2t;
  Mat t_c2t;
  if (useBA) {
    // 使用Bundle Adjustment位姿估计结果
    R_c2t = cameraDetection.baRotationMatrix;
    t_c2t = cameraDetection.baTranslationVector.clone();
    cout << "  [DEBUG] 使用Bundle Adjustment结果进行位姿变换" << endl;
  } else {
    // 使用原始的estimate_tag_pose结果
    R_c2t = cameraDetection.rotationMatrix;
    t_c2t = cameraDetection.translationVector.clone();
    cout << "  [DEBUG] 使用单应矩阵H法进行位姿变换" << endl;
  }
  Mat R_w2t = R_w2c * R_c2t;
  Mat t_w2t = R_w2c * t_c2t + t_w2c;

  // 标签在世界坐标系中的旋转和平移
  worldPose.rotationMatrix = R_w2t;
  worldPose.translationVector = t_w2t;

  // 转换为旋转向量
  Rodrigues(worldPose.rotationMatrix, worldPose.rotationVector);

  cout << "  [DEBUG] 标签在世界坐标系中的位姿：" << endl;
  // cout << "    旋转向量：" << worldPose.rotationVector << endl;
  cout << "    平移向量：" << worldPose.translationVector << endl;

  worldPose.isValid = true;

  return worldPose;
}

// 滑动窗口结构体方法实现
void SlidingWindow::addDetection(const vector<Point2f>& corners,
                                 const Point2f& center) {
  allCorners.push_back(corners);
  allCenters.push_back(center);

  // 保持窗口大小
  if (allCorners.size() > windowSize) {
    allCorners.erase(allCorners.begin());
    allCenters.erase(allCenters.begin());
  }
}

void SlidingWindow::getWindowData(vector<Point2f>& allImagePoints) {
  allImagePoints.clear();

  for (const auto& corners : allCorners) {
    // 扩展图像点和对象点
    for (size_t i = 0; i < corners.size(); ++i) {
      allImagePoints.push_back(corners[i]);
    }
  }
}

/**
 * @brief 使用滑动窗口的PnP位姿估计
 */
AprilTagDetection detectAprilTagsWithSlidingWindow(
    const Mat& image, double tagSize, const Mat& cameraMatrix,
    const Mat& distCoeffs, SlidingWindow& window, double zConstraint,
    int poseMethod) {
  // 首先检测当前帧的AprilTag
  AprilTagDetection currentDetection =
      detectAprilTags(image, tagSize, cameraMatrix, distCoeffs);

  // 将当前检测结果添加到滑动窗口
  window.addDetection(currentDetection.corners, currentDetection.center);

  // 如果窗口不满，返回当前检测结果（使用单帧PnP或原始方法）
  if (!window.isFull() || poseMethod == 1) {
    return currentDetection;
  }

  // 窗口已满，使用窗口内的所有帧进行PnP位姿估计
  vector<Point2f> allImagePoints;
  window.getWindowData(allImagePoints);

  // 使用所有帧的点进行PnP位姿估计
  Mat rvec, tvec;
  rvec = currentDetection.rotationVector.clone();
  tvec = currentDetection.translationVector.clone();
  // 方法2: 使用Bundle Adjustment (BA) 优化位姿
  // 使用滑动窗口内的所有帧进行BA优化
  Mat rvecBA = rvec.clone();
  Mat tvecBA = tvec.clone();

  // 准备数据用于自定义BA实现
  std::vector<FrameObservation> observations;
  std::vector<std::array<double, 6>> poses_initial;

  // 将滑动窗口内的检测结果转换为BA输入格式
  for (const auto& corners : window.allCorners) {
    FrameObservation obs;
    obs.corners_px.reserve(4);

    for (const auto& corner : corners) {
      obs.corners_px.emplace_back(static_cast<double>(corner.x),
                                  static_cast<double>(corner.y));
    }

    observations.push_back(obs);

    // 使用当前PnP结果作为所有帧的初始值
    std::array<double, 6> pose_init;
    pose_init[0] = rvec.at<double>(0, 0);
    pose_init[1] = rvec.at<double>(1, 0);
    pose_init[2] = rvec.at<double>(2, 0);
    pose_init[3] = tvec.at<double>(0, 0);
    pose_init[4] = tvec.at<double>(1, 0);
    pose_init[5] = tvec.at<double>(2, 0);

    poses_initial.push_back(pose_init);
  }

  // 执行自定义Bundle Adjustment优化
  double height_weight = 1000.0;  // 高度约束权重

  std::vector<std::array<double, 6>> poses_optimized;

  if (bundleAdjustmentTagWindow(observations, poses_initial, cameraMatrix,
                                distCoeffs, tagSize, zConstraint, height_weight,
                                poses_optimized)) {
    // 取最后一帧的优化结果
    const auto& optimized_pose = poses_optimized.back();

    // 更新当前检测结果的BA位姿
    rvecBA.at<double>(0, 0) = optimized_pose[0];
    rvecBA.at<double>(1, 0) = optimized_pose[1];
    rvecBA.at<double>(2, 0) = optimized_pose[2];

    tvecBA.at<double>(0, 0) = optimized_pose[3];
    tvecBA.at<double>(1, 0) = optimized_pose[4];
    tvecBA.at<double>(2, 0) = optimized_pose[5];

    currentDetection.baRotationVector = rvecBA.clone();
    currentDetection.baTranslationVector = tvecBA.clone();
    Rodrigues(rvecBA, currentDetection.baRotationMatrix);
    // }

    cout << "  [DEBUG] 使用滑动窗口使用 " << window.allCorners.size()
         << " 帧的数据" << endl;
  }

  return currentDetection;
}

/**
 * @brief 方法4: 使用射线-平面求交法计算 AprilTag 在世界坐标系中的位置
 * @details 实现步骤：
 * 1. 获取 AprilTag 中心点坐标
 * 2. 利用内参去除图像畸变
 * 3.
 * 计算中心点在相机坐标系下的射线向量：利用内参矩阵的逆将像素点转化为归一化相机坐标系下的方向向量
 * 4. 转换到世界坐标系下：利用外参将相机光心 O 和射线方向 d 计算出来
 * 5. 将射线和平面求交点：构造方程 P_target = O + x*d; P_target 的 z
 * 方向分量为固定值，求解 x 并得到 P_target 的 x,y 分量
 * @param cameraDetection 相机坐标系下的 AprilTag 检测结果
 * @param cameraIntrinsics 相机内参
 * @param cameraExtrinsics 相机外参
 * @param zConstraint 世界坐标系中固定的 Z 高度（米）
 * @return 世界坐标系下的 AprilTag 位姿
 */
AprilTagWorldPose rayPlaneIntersectionMethod(
    const AprilTagDetection& cameraDetection,
    const CameraIntrinsics& cameraIntrinsics,
    const CameraExtrinsics& cameraExtrinsics, double zConstraint) {
  AprilTagWorldPose worldPose;
  worldPose.tagId = cameraDetection.tagId;

  // 1. 获取 AprilTag 中心点坐标 (像素坐标)
  Point2f pixelCenter = cameraDetection.center;

  // 2. 利用内参去除图像畸变
  vector<Point2f> distortedPoints = {pixelCenter};
  vector<Point2f> undistortedPoints;

  // 使用相机内参和畸变系数进行去畸变
  undistortPoints(distortedPoints, undistortedPoints,
                  cameraIntrinsics.cameraMatrix,
                  cameraIntrinsics.distortionCoeffs);

  if (undistortedPoints.empty()) {
    cerr << "  [ERROR] 去畸变失败\n";
    return worldPose;
  }

  Point2f undistortedCenter = undistortedPoints[0];

  // 3. 计算中心点在相机坐标系下的射线向量
  // 归一化相机坐标系下的方向向量 (u', v', 1)
  Mat normalizedDir(3, 1, CV_64F);
  normalizedDir.at<double>(0, 0) = undistortedCenter.x;
  normalizedDir.at<double>(1, 0) = undistortedCenter.y;
  normalizedDir.at<double>(2, 0) = 1.0;

  // 相机坐标系下的射线方向 (未归一化)
  Mat cameraDir = cameraIntrinsics.cameraMatrix.inv() * normalizedDir;

  // 归一化相机射线方向
  double norm = sqrt(cameraDir.at<double>(0, 0) * cameraDir.at<double>(0, 0) +
                     cameraDir.at<double>(1, 0) * cameraDir.at<double>(1, 0) +
                     cameraDir.at<double>(2, 0) * cameraDir.at<double>(2, 0));

  if (norm < 1e-8) {
    cerr << "  [ERROR] 射线方向计算失败\n";
    return worldPose;
  }

  cameraDir = cameraDir / norm;

  // 4. 转换到世界坐标系下
  // 获取世界到相机的变换（外参）,世界坐标系原点在相机坐标系中的位姿
  Mat R_c2w = cameraExtrinsics.rotationMatrix;
  Mat t_c2w = cameraExtrinsics.translationVector;

  Mat R_w2c = R_c2w.t();       // 相机→世界 旋转矩阵
  Mat t_w2c = -R_c2w * t_c2w;  // 相机→世界 平移向量 (mm)

  // 相机光心在世界坐标系中的位置 O
  Mat O_world = t_c2w.clone();

  // 射线方向转换到世界坐标系 d_world
  Mat d_world = R_c2w * cameraDir;

  // 5. 将射线和平面求交点
  // 平面方程：P.z = zConstraint (mm)
  double zConstraintMM = zConstraint * 1000;  // 转换为 mm

  // 射线方程：P = O + x*d_world
  // 求解 x: O.z + x*d_world.z = zConstraint
  double denominator = d_world.at<double>(2, 0);

  if (abs(denominator) < 1e-8) {
    cerr << "  [ERROR] 射线与平面平行，无交点\n";
    return worldPose;
  }

  double x = (zConstraintMM - O_world.at<double>(2, 0)) / denominator;

  // 计算目标点 P_target 的 x,y 分量
  Mat P_target(3, 1, CV_64F);
  P_target.at<double>(0, 0) =
      O_world.at<double>(0, 0) + x * d_world.at<double>(0, 0);
  P_target.at<double>(1, 0) =
      O_world.at<double>(1, 0) + x * d_world.at<double>(1, 0);
  P_target.at<double>(2, 0) = zConstraintMM;

  // 使用原始旋转矩阵（从方法1或其他方法获取），仅更新平移向量
  worldPose.rotationMatrix = R_c2w * cameraDetection.rotationMatrix;
  worldPose.translationVector = P_target.clone();

  // 转换为旋转向量
  Rodrigues(worldPose.rotationMatrix, worldPose.rotationVector);

  worldPose.isValid = true;

  cout << "  [DEBUG] 使用位姿估计方法4: Ray-Plane Intersection\n";

  return worldPose;
}

/**
 * @brief 执行Bundle Adjustment优化
 */
bool bundleAdjustmentTagWindow(
    const std::vector<FrameObservation>& observations,
    const std::vector<std::array<double, 6>>& poses_initial,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double tag_size_m,
    double desired_tag_z, double height_weight,
    std::vector<std::array<double, 6>>& poses_optimized) {
  if (observations.empty() || poses_initial.empty()) {
    cerr << "No observations or initial poses provided\n";
    return false;
  }

  const int N = static_cast<int>(observations.size());
  if (N != poses_initial.size()) {
    cerr << "Number of observations and initial poses mismatch\n";
    return false;
  }

  // Camera intrinsics
  double fx = cameraMatrix.at<double>(0, 0);
  double fy = cameraMatrix.at<double>(1, 1);
  double cx = cameraMatrix.at<double>(0, 2);
  double cy = cameraMatrix.at<double>(1, 2);

  // Distortion coefficients
  std::array<double, 5> dist = {0, 0, 0, 0, 0};
  if (distCoeffs.rows >= 5) {
    for (int i = 0; i < 5; ++i) {
      dist[i] = distCoeffs.at<double>(i, 0);
    }
  }

  // Build the problem
  ceres::Problem problem;

  // Parameter blocks: for each frame we have angle-axis (3) and translation (3)
  poses_optimized = poses_initial;

  std::vector<std::array<double, 3>> angle_axis_params(N);
  std::vector<std::array<double, 3>> trans_params(N);

  for (int i = 0; i < N; ++i) {
    // initialize with provided initial values
    angle_axis_params[i][0] = poses_optimized[i][0];
    angle_axis_params[i][1] = poses_optimized[i][1];
    angle_axis_params[i][2] = poses_optimized[i][2];

    trans_params[i][0] = poses_optimized[i][3];
    trans_params[i][1] = poses_optimized[i][4];
    trans_params[i][2] = poses_optimized[i][5];

    // Add parameter blocks to problem
    problem.AddParameterBlock(angle_axis_params[i].data(), 3);
    problem.AddParameterBlock(trans_params[i].data(), 3);
  }

  // Prepare object points for a tag (same for all frames)
  std::vector<cv::Point3d> tag_corners = MakeTagCorners3D(tag_size_m);

  // Add reprojection residuals for each observed corner in each frame
  for (int i = 0; i < N; ++i) {
    const auto& obs = observations[i];
    if (obs.corners_px.size() != tag_corners.size()) {
      cerr << "Frame " << i << " has unexpected corners count\n";
      return false;
    }

    for (size_t k = 0; k < tag_corners.size(); ++k) {
      const cv::Point2d observed = obs.corners_px[k];
      const cv::Point3d pt3 = tag_corners[k];

      ceres::CostFunction* cost =
          ReprojectionError::Create(observed, pt3, fx, fy, cx, cy, dist);
      // robust loss
      ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

      problem.AddResidualBlock(cost, loss, angle_axis_params[i].data(),
                               trans_params[i].data());
    }

    // Add height constraint residual on translation z
    ceres::CostFunction* height_cost =
        HeightConstraint::Create(desired_tag_z, height_weight);
    problem.AddResidualBlock(height_cost, nullptr, trans_params[i].data());
  }

  // Solver options
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::DENSE_SCHUR;  // small problem -> ok
  options.minimizer_progress_to_stdout = false;     // 关闭输出以保持简洁
  options.num_threads = 4;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Update the optimized poses
  for (int i = 0; i < N; ++i) {
    poses_optimized[i][0] = angle_axis_params[i][0];
    poses_optimized[i][1] = angle_axis_params[i][1];
    poses_optimized[i][2] = angle_axis_params[i][2];

    poses_optimized[i][3] = trans_params[i][0];
    poses_optimized[i][4] = trans_params[i][1];
    poses_optimized[i][5] = trans_params[i][2];
  }

  return true;
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

  // 计算范围
  double rangeX = maxX - minX;
  double rangeY = maxY - minY;
  // double rangeZ = maxZ - minZ;

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
  bool drawMode = false;      // 默认关闭绘制功能
  int poseMethod = 1;         // 默认使用方法1: estimate_tag_pose()

  // 解析可选参数
  for (int i = 5; i < argc; ++i) {
    if (strcmp(argv[i], "--draw") == 0) {
      drawMode = true;
    } else if (strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      return 0;
    } else if (i == 5 &&
               (strcmp(argv[i], "--pnp") != 0 &&
                strcmp(argv[i], "--ba") != 0)) {  // 第五个参数是zConstraint
      zConstraint = atof(argv[i]);
    } else if (strcmp(argv[i], "--method1") == 0) {
      poseMethod = 1;  // 使用原始的estimate_tag_pose()
    } else if (strcmp(argv[i], "--method2") == 0 ||
               strcmp(argv[i], "--ba") == 0) {
      poseMethod = 2;  // 使用BA结果
    } else if (strcmp(argv[i], "--method3") == 0 ||
               strcmp(argv[i], "--ray") == 0) {
      poseMethod = 3;  // 使用射线平面相交法结果
    }
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

  // 创建滑动窗口，窗口大小为3帧
  const int windowSize = 10;
  SlidingWindow slidingWindow(windowSize);

  for (const string& imageFile : imageFiles) {
    cout << "  处理图像: " << imageFile << endl;

    Mat image = imread(imageFile);
    if (image.empty()) {
      cerr << "  [ERROR] 无法读取图像: " << imageFile << endl;
      continue;
    }

    double old_cx = intrinsics.cameraMatrix.at<double>(0, 2);
    double old_cy = intrinsics.cameraMatrix.at<double>(1, 2);
    double x_offset = 60;
    double y_offset = 450;
    double new_cx = old_cx - x_offset;
    double new_cy = old_cy - y_offset;
    Mat cameraMatrix_cropped = intrinsics.cameraMatrix.clone();
    cameraMatrix_cropped.at<double>(0, 2) = new_cx;
    cameraMatrix_cropped.at<double>(1, 2) = new_cy;

    // 检测 AprilTag 使用滑动窗口 (暂时使用原始相机矩阵，未裁切)
    AprilTagDetection detection = detectAprilTagsWithSlidingWindow(
        image, tagSize, cameraMatrix_cropped, intrinsics.distortionCoeffs,
        slidingWindow, zConstraint, poseMethod);

    // 保存检测到AprilTag的图像
    if (detection.isValid) {
      // 创建detect目录
      string detectDir = imageFolder + "/detect";
      fs::create_directory(detectDir);

      // 提取原始文件名并构造保存路径
      string basename = fs::path(imageFile).filename().string();
      string savePath = detectDir + "/" + basename;

      // 如果处于绘制模式，则在图像上绘制AprilTag的四个角点和中心点
      if (drawMode) {
        // 绘制四个角点
        for (int i = 0; i < 4; ++i) {
          circle(image, detection.corners[i], 5, Scalar(0, 255, 0),
                 -1);  // 绿色圆点标记角点
        }
        // 绘制中心点
        circle(image, detection.center, 8, Scalar(255, 0, 0),
               -1);  // 蓝色圆点标记中心点
                     // 保存图像
        imwrite(savePath, image);
        cout << "  [成功] 检测图像已保存到: " << savePath << endl;
      }
    }

    if (!detection.isValid) {
      cout << "  [信息] 未检测到 AprilTag\n";
      continue;
    }

    // 变换到世界坐标系 - 根据选择的方法使用相应的位姿结果
    vector<AprilTagWorldPose> worldPoses;
    AprilTagWorldPose worldPose;

    switch (poseMethod) {
      case 1:
        worldPose = transformToWorldCoordinates(
            detection, extrinsics,
            false);  // 使用原始的estimate_tag_pose结果
        cout << "  [DEBUG] 使用位姿估计方法1: estimate_tag_pose()\n";
        break;

      case 2:
        worldPose = transformToWorldCoordinates(detection, extrinsics,
                                                true);  // 使用BA结果
        cout << "  [DEBUG] 使用位姿估计方法2: Bundle Adjustment\n";
        break;

      case 3:
        // 使用射线-平面求交法，该方法本身需要强制Z约束
        if (zConstraint <= 0.0) {
          cerr << "  [ERROR] 使用方法3 (Ray-Plane Intersection) "
                  "必须提供Z高度约束\n";
          continue;  // 跳过该帧
        }
        worldPose = rayPlaneIntersectionMethod(detection, intrinsics,
                                               extrinsics, zConstraint);
        break;

      default:
        worldPose = transformToWorldCoordinates(detection, extrinsics,
                                                false);  // 默认使用方法1
        cout << "  [DEBUG] 使用默认位姿估计方法1: estimate_tag_pose()\n";
        break;
    }

    worldPoses.push_back(worldPose);
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
