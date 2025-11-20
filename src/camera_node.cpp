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
#include <utility>

#include "MvCameraControl.h"
#include "rclcpp/serialization.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/writer.hpp"

class CameraNode : public rclcpp::Node {
 public:
  CameraNode() : Node("camera_node") {
    // 声明参数
    this->declare_parameter<std::string>("image_save_path",
                                         "/home/zzh/Pictures/hik");
    this->declare_parameter<int>("capture_fps", 30);  // 默认30fps

    // 获取参数值
    image_save_path_ = this->get_parameter("image_save_path").as_string();
    capture_fps_ = this->get_parameter("capture_fps").as_int();

    // 限制帧率只能是30或60fps
    if (capture_fps_ != 30 && capture_fps_ != 60) {
      RCLCPP_WARN(this->get_logger(), "Invalid fps %d, using default 30fps",
                  capture_fps_);
      capture_fps_ = 30;
    }

    // 计算帧间隔（毫秒）
    frame_interval_ms_ = (capture_fps_ > 0) ? (1000 / capture_fps_) : 0;
    last_frame_time_ = std::chrono::system_clock::now();

    // 日志输出参数
    RCLCPP_INFO(
        this->get_logger(), "Image save path: %s",
        image_save_path_.empty() ? "(disabled)" : image_save_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target FPS: %d", capture_fps_);

    // 创建Publisher
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // 初始化rosbag
    initializeRosbag();

    // 初始化海康相机
    initializeCamera();

    // 启动图像采集
    startGrabbing();
  }

  ~CameraNode() {
    // 停止图像采集
    std::cout << "start stop grab" << std::endl;
    stopGrabbing();

    // 关闭rosbag
    closeRosbag();
    std::cout << "close rosbag" << std::endl;

    // 关闭相机
    closeCamera();
    std::cout << "close camera" << std::endl;

    // 反初始化SDK
    MV_CC_Finalize();
    std::cout << "finalize SDK" << std::endl;
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

    // 如果打开失败，可能是相机已被占用或其他原因
    if (nRet != MV_OK) {
      // 输出错误码并尝试再次打开（不切换模式）
      RCLCPP_WARN(this->get_logger(),
                  "Failed to open camera: 0x%x. Retrying...", nRet);

      // 尝试再次打开（最多3次重试）
      int retry_count = 0;
      const int max_retries = 3;
      while (nRet != MV_OK && retry_count < max_retries) {
        retry_count++;
        RCLCPP_WARN(this->get_logger(), "Retrying to open camera (%d/%d)...",
                    retry_count, max_retries);
        nRet = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);
      }

      if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open camera after %d retries.", max_retries);
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Camera opened successfully.");

    nRet =
        MV_CC_SetEnumValue(handle_, "TriggerMode", 0);  // Set trigger mode off
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set trigger mode.");
      return;
    }

    // ========================================================================
    // 设置帧率
    // ========================================================================
    MVCC_FLOATVALUE stFrameRateValue = {0};
    nRet =
        MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &stFrameRateValue);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(),
                  "Current FrameRate: %.2f Hz, Range: [%.2f, %.2f] Hz",
                  stFrameRateValue.fCurValue, stFrameRateValue.fMin,
                  stFrameRateValue.fMax);

      // 设置帧率为30或60fps
      float target_frame_rate = static_cast<float>(capture_fps_);

      // 确保目标帧率在有效范围内
      if (target_frame_rate < stFrameRateValue.fMin) {
        target_frame_rate = stFrameRateValue.fMin;
        RCLCPP_WARN(this->get_logger(),
                    "Target frame rate %.2f Hz is below minimum, using %.2f Hz",
                    static_cast<float>(capture_fps_), target_frame_rate);
      }

      if (target_frame_rate > stFrameRateValue.fMax) {
        target_frame_rate = stFrameRateValue.fMax;
        RCLCPP_WARN(this->get_logger(),
                    "Target frame rate %.2f Hz is above maximum, using %.2f Hz",
                    static_cast<float>(capture_fps_), target_frame_rate);
      }

      // 设置帧率
      nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
                                 target_frame_rate);
      if (nRet == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRate set to: %.2f Hz",
                    target_frame_rate);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set frame rate: 0x%x",
                     nRet);
      }
    }

    // ========================================================================
    // 设置增益
    // ========================================================================
    MVCC_FLOATVALUE stGainValue = {0};
    nRet = MV_CC_GetFloatValue(handle_, "Gain", &stGainValue);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(),
                  "Current Gain: %.2f dB, Range: [%.2f, %.2f] dB",
                  stGainValue.fCurValue, stGainValue.fMin, stGainValue.fMax);

      // 计算目标增益：增加增益以提高图像亮度
      float targetGain = stGainValue.fCurValue + 5.0;  // 增加5dB增益

      // 确保目标增益在有效范围内
      if (targetGain > stGainValue.fMax) {
        targetGain = stGainValue.fMax;
        RCLCPP_WARN(
            this->get_logger(),
            "Target gain (%.2f dB) exceeds maximum. Clamping to %.2f dB",
            targetGain, stGainValue.fMax);
      }

      nRet = MV_CC_SetFloatValue(handle_, "Gain", targetGain);
      if (nRet == MV_OK) {
        RCLCPP_INFO(this->get_logger(),
                    "Gain set to: %.2f dB (increased by 5 dB from baseline)",
                    targetGain);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set gain: 0x%x", nRet);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to query gain parameter: 0x%x",
                  nRet);
    }

    // ========================================================================
    // 设置曝光和帧率参数（需要注意参数依赖顺序）
    // ========================================================================

    // Step 1: 禁用自动曝光（重要！某些相机需要先禁用自动模式）
    nRet = MV_CC_SetEnumValue(handle_, "ExposureAuto", 1);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Auto Exposure disabled");
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "Failed to disable auto exposure: 0x%x (may not be available)", nRet);
    }

    // Step 2: 查询曝光时间范围
    MVCC_FLOATVALUE stExposureTime = {0};
    nRet = MV_CC_GetFloatValue(handle_, "ExposureTime", &stExposureTime);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(),
                  "Current ExposureTime: %.2f μs, Range: [%.2f, %.2f] μs",
                  stExposureTime.fCurValue, stExposureTime.fMin,
                  stExposureTime.fMax);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to query exposure time parameter: 0x%x", nRet);
    }

    // Step 3: 查询帧率范围
    stFrameRateValue = {0};
    nRet =
        MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &stFrameRateValue);
    if (nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to query frame rate parameter: 0x%x", nRet);
    }

    // Step 4: 先设置合理的曝光时间（在当前帧率的约束下）
    if (nRet == MV_OK && stExposureTime.fMin > 0) {
      // 使用当前帧率计算最大可能的曝光时间
      float currentFrameRate =
          stFrameRateValue.fCurValue > 0 ? stFrameRateValue.fCurValue : 30.0f;
      float frameIntervalUs = 1000000.0f / currentFrameRate;

      // 曝光时间为帧间隔的 90%（留出一些裕度）
      float targetExposureTime = frameIntervalUs * 0.9f;

      // 确保在有效范围内
      if (targetExposureTime < stExposureTime.fMin) {
        targetExposureTime = stExposureTime.fMin;
      }
      if (targetExposureTime > stExposureTime.fMax) {
        targetExposureTime = stExposureTime.fMax;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Calculated exposure time: %.2f μs (frame rate: %.2f Hz, "
                  "interval: %.2f μs)",
                  targetExposureTime, currentFrameRate, frameIntervalUs);

      nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", targetExposureTime);
      if (nRet == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "ExposureTime set to: %.2f μs",
                    targetExposureTime);
      } else {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to set exposure time: 0x%x\n"
            "  Hint: Check if ExposureAuto or ExposureMode needs adjustment",
            nRet);
        // 不中断初始化流程，继续尝试后续参数设置
      }
    }

    nRet = MV_CC_RegisterImageCallBackEx2(handle_, &CameraNode::ImageCallback,
                                          this, true);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to register image callback.");
      return;
    }
  }

  void startGrabbing() {
    int nRet = MV_CC_StartGrabbing(handle_);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start grabbing.");
    }
  }

  void stopGrabbing() {
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

  // 回调函数
  static void __stdcall ImageCallback(MV_FRAME_OUT* pstFrame, void* pUser,
                                      bool bAutoFree) {
    // Convert raw frame to a compressed image via SDK, then decode with OpenCV
    CameraNode* node = static_cast<CameraNode*>(pUser);
    if (pstFrame == nullptr || node == nullptr) {
      return;
    }

    // 记录回调被调用的次数和时间（用于调试帧率问题）
    static int frame_count = 0;
    static auto start_time = std::chrono::system_clock::now();

    frame_count++;
    auto now = std::chrono::system_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(now - start_time)
            .count();

    if (elapsed >= 1.0) {  // 每1秒打印一次帧率
      RCLCPP_INFO(node->get_logger(), "Callback FPS: %d", frame_count);
      frame_count = 0;
      start_time = now;
    }

    // Prepare output buffer for conversion (estimate)
    unsigned int nWidth = pstFrame->stFrameInfo.nWidth;
    unsigned int nHeight = pstFrame->stFrameInfo.nHeight;
    unsigned int nFrameLen = pstFrame->stFrameInfo.nFrameLen;
    unsigned int nOutputBufferSize = nWidth * nHeight * 4 + 1024;
    unsigned char* pOutputBuffer = new unsigned char[nOutputBufferSize];

    MV_SAVE_IMAGE_PARAM_EX3 stSaveParam;
    memset(&stSaveParam, 0, sizeof(stSaveParam));
    stSaveParam.pData = (unsigned char*)pstFrame->pBufAddr;
    stSaveParam.nDataLen = nFrameLen;
    stSaveParam.enPixelType = pstFrame->stFrameInfo.enPixelType;
    stSaveParam.nWidth = nWidth;
    stSaveParam.nHeight = nHeight;
    stSaveParam.pImageBuffer = pOutputBuffer;
    stSaveParam.nBufferSize = nOutputBufferSize;
    // Use JPEG conversion for smaller, decodable buffer
    stSaveParam.enImageType = MV_Image_Jpeg;
    stSaveParam.nJpgQuality = 95;
    stSaveParam.iMethodValue = 2;

    int nRet = MV_CC_SaveImageEx3(node->handle_, &stSaveParam);
    if (nRet != MV_OK || stSaveParam.nImageLen == 0) {
      // Conversion failed; cleanup and return
      delete[] pOutputBuffer;
      // Free frame buffer if required
      if (!bAutoFree) {
        MV_CC_FreeImageBuffer(node->handle_, pstFrame);
      }
      return;
    }

    // Decode JPEG buffer into OpenCV Mat
    std::vector<unsigned char> imgBuf(
        stSaveParam.pImageBuffer,
        stSaveParam.pImageBuffer + stSaveParam.nImageLen);
    cv::Mat img = cv::imdecode(imgBuf, cv::IMREAD_COLOR);

    // 释放SDK转换后的数据
    delete[] pOutputBuffer;

    if (!img.empty()) {
      // 保存到rosbag（优先处理rosbag，减少延迟）
      if (node->rosbag_writer_ && node->serializer_) {
        try {
          // 直接使用转换后的图像数据，不使用中间消息
          auto cv_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img)
                            .toImageMsg();
          cv_msg->header.stamp = node->get_clock()->now();
          cv_msg->header.frame_id = "camera_frame";

          // 保存到rosbag
          node->rosbag_writer_->write(*cv_msg, "camera/image_raw",
                                      node->get_clock()->now());

          // 发布图像消息
          node->publisher_->publish(*cv_msg);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(), "Failed to write to rosbag: %s",
                       e.what());
        }
      }
    }

    if (!bAutoFree) {
      // If SDK did not auto free, free using device handle
      MV_CC_FreeImageBuffer(node->handle_, pstFrame);
    }
  }

  // Rosbag related functions
  void initializeRosbag() {
    // 创建rosbag目录
    std::string rosbag_dir = image_save_path_ + "/rosbag";

    try {
      std::filesystem::create_directories(rosbag_dir);
    } catch (const std::filesystem::filesystem_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create rosbag directory: %s",
                   e.what());
      return;
    }

    // 生成rosbag文件名（当前时间）
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);
    std::ostringstream bag_filename;
    bag_filename << std::put_time(&now_tm, "%Y%m%d_%H%M%S") << ".db3";

    std::string bag_file_path = rosbag_dir + "/" + bag_filename.str();

    RCLCPP_INFO(this->get_logger(), "Rosbag will be saved to: %s",
                bag_file_path.c_str());

    // 初始化rosbag writer
    try {
      rosbag_writer_ = std::make_unique<rosbag2_cpp::Writer>();

      // 存储选项
      rosbag2_cpp::StorageOptions storage_options;
      storage_options.uri = bag_file_path;
      storage_options.storage_id = "sqlite3";

      // 配置选项
      rosbag2_cpp::ConverterOptions converter_options;

      // 打开rosbag
      rosbag_writer_->open(storage_options, converter_options);

      // 创建序列化器
      serializer_ =
          std::make_unique<rclcpp::Serialization<sensor_msgs::msg::Image>>();

      RCLCPP_INFO(this->get_logger(), "Rosbag initialized successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize rosbag: %s",
                   e.what());
      return;
    }
  }

  void closeRosbag() {
    if (rosbag_writer_) {
      try {
        rosbag_writer_->close();
        RCLCPP_INFO(this->get_logger(), "Rosbag closed successfully");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to close rosbag: %s",
                     e.what());
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  void* handle_ = nullptr;

  // Rosbag related members
  std::unique_ptr<rosbag2_cpp::Writer> rosbag_writer_;
  std::unique_ptr<rclcpp::Serialization<sensor_msgs::msg::Image>> serializer_;

  // 参数成员
  std::string image_save_path_;
  int capture_fps_;
  int frame_interval_ms_;
  std::chrono::system_clock::time_point last_frame_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
