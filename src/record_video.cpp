#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <thread>
#include <utility>

#include "MvCameraControl.h"
#include "rclcpp/serialization.hpp"

class VideoRecorder : public rclcpp::Node {
 public:
  VideoRecorder() : Node("video_recorder") {
    // 声明参数
    this->declare_parameter<std::string>("video_save_path",
                                         "/home/zzh/Pictures/hik");

    // 获取参数值
    video_save_path_ = this->get_parameter("video_save_path").as_string();

    // 日志输出参数
    RCLCPP_INFO(this->get_logger(), "Video save path: %s",
                video_save_path_.c_str());

    // 初始化海康相机
    initializeCamera();

    // 启动视频录制
    startRecording();
  }

  ~VideoRecorder() {
    // 停止视频录制
    stopRecording();

    // 关闭相机
    closeCamera();

    // 反初始化SDK
    MV_CC_Finalize();

    std::cout << "Video recorder cleaned up" << std::endl;
  }

 private:
  void initializeCamera() {
    int nRet = MV_CC_Initialize();
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera SDK.");
      return;
    }

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
      RCLCPP_ERROR(this->get_logger(), "No devices found.");
      return;
    }

    MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[0]);

    // 尝试打开相机，先使用独占模式
    nRet = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);

    // 如果打开失败，尝试再次打开
    if (nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to open camera: 0x%x", nRet);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Camera opened successfully.");

    // 设置与camera_node相同的参数
    unsigned int roi_width = 2384;
    unsigned int roi_height = 1584;
    unsigned int roi_offset_x = 60;
    unsigned int roi_offset_y = 464;

    // 设置ROI水平偏移
    nRet = MV_CC_SetIntValueEx(handle_, "OffsetX", roi_offset_x);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set ROI OffsetX: 0x%x", nRet);
    }

    // 设置ROI垂直偏移
    nRet = MV_CC_SetIntValueEx(handle_, "OffsetY", roi_offset_y);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set ROI OffsetY: 0x%x", nRet);
    }

    // 设置ROI宽度
    nRet = MV_CC_SetIntValueEx(handle_, "Width", roi_width);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set ROI Width: 0x%x", nRet);
    }

    // 设置ROI高度
    nRet = MV_CC_SetIntValueEx(handle_, "Height", roi_height);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set ROI Height: 0x%x", nRet);
    }

    // 关闭自动曝光模式
    nRet = MV_CC_SetEnumValue(handle_, "ExposureAuto", 1);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Auto Exposure disabled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to disable auto exposure: 0x%x",
                   nRet);
    }

    // 关闭触发模式
    nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set trigger mode: 0x%x",
                   nRet);
      return;
    }

    // 设置采集帧率为24fps
    nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", 24.0f);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRate set to: 24.0 Hz");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set frame rate: 0x%x", nRet);
    }
  }

  void startRecording() {
    int nRet = MV_CC_StartGrabbing(handle_);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start grabbing: 0x%x", nRet);
      return;
    }

    // 获取PayloadSize
    uint64_t nPayloadSize = 0;
    unsigned int nAlignment = 0;
    nRet = MV_CC_GetPayloadSize(handle_, &nPayloadSize, &nAlignment);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get payload size: 0x%x",
                   nRet);
      return;
    }

    // 分配缓冲区
    unsigned char* pData = new unsigned char[nPayloadSize];
    if (pData == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate buffer.");
      return;
    }

    // 创建视频编写器
    std::string video_filename = getCurrentTime() + ".avi";
    std::string video_path = video_save_path_ + "/video/" + video_filename;

    // 获取图像尺寸
    MVCC_INTVALUE_EX width_val = {0};
    MVCC_INTVALUE_EX height_val = {0};
    MV_CC_GetIntValueEx(handle_, "Width", &width_val);
    MV_CC_GetIntValueEx(handle_, "Height", &height_val);

    cv::VideoWriter video_writer;
    video_writer.open(video_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                      24, cv::Size(width_val.nCurValue, height_val.nCurValue));

    if (!video_writer.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video writer.");
      delete[] pData;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Video recording started: %s",
                video_path.c_str());

    // 录制循环
    while (rclcpp::ok()) {
      MV_FRAME_OUT_INFO_EX stFrameInfo = {0};

      // 获取一帧图像
      nRet = MV_CC_GetOneFrameTimeout(
          handle_, pData, (unsigned int)nPayloadSize, &stFrameInfo, 1000);

      if (nRet == MV_OK) {
        // 转换图像数据
        cv::Mat raw_mat(static_cast<int>(stFrameInfo.nHeight),
                        static_cast<int>(stFrameInfo.nWidth), CV_8UC1, pData);
        cv::Mat color_mat;
        cv::cvtColor(raw_mat, color_mat, cv::COLOR_GRAY2BGR);

        // 写入视频
        video_writer.write(color_mat);
      } else {
        // 如果获取失败，等待一段时间再重试
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    // 释放资源
    video_writer.release();
    delete[] pData;

    RCLCPP_INFO(this->get_logger(), "Video recording stopped");
  }

  void stopRecording() {
    int nRet = MV_CC_StopGrabbing(handle_);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to stop grabbing.");
    }
  }

  void closeCamera() {
    int nRet = MV_CC_CloseDevice(handle_);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to close camera.");
    }

    MV_CC_DestroyHandle(handle_);
  }

  std::string getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");

    return oss.str();
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  void* handle_ = nullptr;

  // 参数成员
  std::string video_save_path_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoRecorder>());
  rclcpp::shutdown();
  return 0;
}
