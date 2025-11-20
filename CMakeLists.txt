cmake_minimum_required(VERSION 3.10)
project(cam_intrinsic_calib)
set(CMAKE_CXX_STANDARD 17)  # 开启 C++17 支持

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)  # 如果使用 OpenCV 处理图像，确保已找到 OpenCV

# 设置 AprilTag 和 海康 SDK 的 include 路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # AprilTag 头文件和源文件
  /opt/MVS/include  # 这里替换为实际的海康 SDK 的路径
)

# 设置海康 SDK 的库文件路径
link_directories(
  /opt/MVS/lib  # 替换为实际的路径
)

# 可执行文件1: camera_node (ROS2 节点)
add_executable(camera_node src/camera_node.cpp)

# 添加依赖的库
target_link_libraries(camera_node
  /opt/MVS/lib/64/libMvCameraControl.so  # 根据实际路径链接海康的库文件
)

# 将 target 与依赖项关联
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV  # 如果使用 OpenCV 处理图像，添加 OpenCV 依赖
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
# 编译 AprilTag 源文件
add_library(apriltag_lib STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/apriltag.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/apriltag_pose.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/apriltag_quad_thresh.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tag16h5.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tag25h9.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tag36h10.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tag36h11.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tagCircle21h7.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tagCircle49h12.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tagCustom48h12.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tagStandard41h12.c
  ${CMAKE_CURRENT_SOURCE_DIR}/include/tagStandard52h13.c
)

# 为 apriltag 库添加 include 目录
target_include_directories(apriltag_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(apriltag_detector src/apriltag_detector.cpp)

# 为 apriltag_detector 链接 OpenCV 和 apriltag 库
target_link_libraries(apriltag_detector
  ${OpenCV_LIBS}
  apriltag_lib
  m
)

# 安装可执行文件
install(TARGETS camera_node calibrate_camera undistort_camera extrinsic_calib apriltag_detector
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 可执行文件安装到 lib/${PROJECT_NAME} 目录
)

# 生成 ament 包元数据（确保没有漏掉这个，生成相应的 setup.bash 文件）
ament_package()
