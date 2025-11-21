cmake_minimum_required(VERSION 3.10)
project(cam_intrinsic_calib)
set(CMAKE_CXX_STANDARD 17)  # 开启 C++17 支持

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)  # 如果使用 OpenCV 处理图像，确保已找到 OpenCV
find_package(rosbag2_cpp REQUIRED)  # 添加 rosbag2_cpp 依赖

# 设置海康 SDK 的 include 路径
include_directories(
  /opt/MVS/include  # 这里替换为实际的海康 SDK 的路径
)

# 设置海康 SDK 的库文件路径
link_directories(
  /opt/MVS/lib  # 替换为实际的路径
)

# 可执行文件1: camera_node (独立相机节点)
add_executable(camera_node src/camera_node.cpp)

# 为 camera_node 链接 OpenCV 和海康库
target_link_libraries(camera_node
  ${OpenCV_LIBS}
  /opt/MVS/lib/64/libMvCameraControl.so  # 根据实际路径链接海康的库文件
)
# 可执行文件2: calibrate_camera (独立标定工具)
add_executable(calibrate_camera src/calibrate_camera.cpp)

# 为 calibrate_camera 链接 OpenCV
target_link_libraries(calibrate_camera
  ${OpenCV_LIBS}
)

# 可执行文件3: undistort_camera (图像去畸变工具)
add_executable(undistort_camera src/undistort_camera.cpp)

# 为 undistort_camera 链接 OpenCV
target_link_libraries(undistort_camera
  ${OpenCV_LIBS}
)

# 可执行文件4: extrinsic_calib (外参标定工具)
add_executable(extrinsic_calib src/extrinsic_calib.cpp)

# 为 extrinsic_calib 链接 OpenCV
target_link_libraries(extrinsic_calib
  ${OpenCV_LIBS}
)

# 可执行文件5: apriltag_detector (AprilTag 检测和位姿求解工具)
# 链接系统安装的 apriltag 库
find_package(apriltag REQUIRED)

add_executable(apriltag_detector src/apriltag_detector.cpp)

# 为 apriltag_detector 链接 OpenCV 和系统 apriltag 库
target_link_libraries(apriltag_detector
  ${OpenCV_LIBS}
  apriltag
  m
)

# 可执行文件6: record_video (视频录制工具)
add_executable(record_video src/record_video.cpp)

# 添加依赖的库
target_link_libraries(record_video
  /opt/MVS/lib/64/libMvCameraControl.so  # 根据实际路径链接海康的库文件
)

# 将 target 与依赖项关联
ament_target_dependencies(record_video
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# 安装可执行文件
install(TARGETS camera_node calibrate_camera undistort_camera extrinsic_calib apriltag_detector record_video
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 可执行文件安装到 lib/${PROJECT_NAME} 目录
)

# 生成 ament 包元数据（确保没有漏掉这个，生成相应的 setup.bash 文件）
ament_package()
