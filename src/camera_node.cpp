#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <std_msgs/msg/header.hpp>

#include "MvCameraControl.h"

class CameraNode : public rclcpp::Node {
 public:
  CameraNode() : Node("camera_node") {
    // 声明参数
    this->declare_parameter<std::string>("image_save_path",
                                         "/home/zzh/Pictures/hik");
    this->declare_parameter<int>("capture_fps", 3);

    // 获取参数值
    image_save_path_ = this->get_parameter("image_save_path").as_string();
    capture_fps_ = this->get_parameter("capture_fps").as_int();

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

    // 初始化海康相机
    initializeCamera();

    // 启动图像采集
    startGrabbing();
  }

  ~CameraNode() {
    // 停止图像采集
    std::cout << "start stop grab" << std::endl;
    stopGrabbing();

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
    // Open device with exclusive access (matches SimpleCapture example)
    nRet = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
      return;
    }

    nRet =
        MV_CC_SetEnumValue(handle_, "TriggerMode", 0);  // Set trigger mode off
    if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set trigger mode.");
      return;
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

      // 计算目标增益：当前增益 + 增量
      float targetGain = stGainValue.fCurValue - 5.0;

      // 确保目标增益在有效范围内
      if (targetGain > stGainValue.fMax) {
        targetGain = stGainValue.fMax;
        RCLCPP_WARN(
            this->get_logger(),
            "Target gain (%.2f dB) exceeds maximum. Clamping to %.2f dB",
            stGainValue.fCurValue, targetGain);
      }

      nRet = MV_CC_SetFloatValue(handle_, "Gain", targetGain);
      if (nRet == MV_OK) {
        RCLCPP_INFO(this->get_logger(),
                    "Gain set to: %.2f dB (increased by 10 dB from baseline)",
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

    // Step 2: 设置曝光模式为手动（如果支持）
    // nRet = MV_CC_SetEnumValue(handle_, "ExposureMode", 0);  // 0 = Manual
    // if (nRet == MV_OK) {
    //   RCLCPP_INFO(this->get_logger(), "Exposure mode set to Manual");
    // } else {
    //   RCLCPP_WARN(this->get_logger(),
    //               "Failed to set exposure mode: 0x%x (may not be available)",
    //               nRet);
    // }

    // Step 3: 查询曝光时间范围
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

    // Step 4: 查询帧率范围
    MVCC_FLOATVALUE stFrameRateValue = {0};
    nRet =
        MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &stFrameRateValue);
    if (nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to query frame rate parameter: 0x%x", nRet);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Current FrameRate: %.2f Hz, Range: [%.2f, %.2f] Hz",
                  stFrameRateValue.fCurValue, stFrameRateValue.fMin,
                  stFrameRateValue.fMax);
    }

    // Step 5: 先设置合理的曝光时间（在当前帧率的约束下）
    if (nRet == MV_OK && stExposureTime.fMin > 0) {
      // 使用当前帧率或默认帧率计算合理的曝光时间
      float currentFrameRate =
          stFrameRateValue.fCurValue > 0 ? stFrameRateValue.fCurValue : 30.0f;
      float frameIntervalUs = 1000000.0f / currentFrameRate;

      // 曝光时间为帧间隔的 80%（留出较多裕度）
      float targetExposureTime = frameIntervalUs * 0.8f;

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

    // Step 6: 然后设置为最大帧率
    if (nRet == MV_OK && stFrameRateValue.fMax > 0) {
      float maxFrameRate = stFrameRateValue.fMax;
      nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", maxFrameRate);
      if (nRet == MV_OK) {
        RCLCPP_INFO(this->get_logger(),
                    "AcquisitionFrameRate set to maximum: %.2f Hz",
                    maxFrameRate);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set maximum frame rate: 0x%x", nRet);
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

    // 帧率控制：检查是否应该处理此帧
    if (node->frame_interval_ms_ > 0) {
      auto now = std::chrono::system_clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - node->last_frame_time_)
                            .count();
      if (elapsed_ms < node->frame_interval_ms_) {
        // 帧间隔未达，释放缓冲后返回
        if (!bAutoFree) {
          MV_CC_FreeImageBuffer(node->handle_, pstFrame);
        }
        return;
      }
      node->last_frame_time_ = now;
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
    if (!img.empty()) {
      // 如果指定了保存路径，保存图像到本地
      if (!node->image_save_path_.empty()) {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_c);
        std::ostringstream file_path;
        file_path << node->image_save_path_ << "/"
                  << std::put_time(&now_tm, "%Y%m%d_%H%M%S") << "_"
                  << pstFrame->stFrameInfo.nFrameNum << ".jpg";
        cv::imwrite(file_path.str(), img);
      }

      // Fill header and publish
      auto cv_msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
      cv_msg->header.stamp = node->get_clock()->now();
      node->publisher_->publish(*cv_msg);
    }

    delete[] pOutputBuffer;

    if (!bAutoFree) {
      // If SDK did not auto free, free using device handle
      MV_CC_FreeImageBuffer(node->handle_, pstFrame);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  void* handle_ = nullptr;

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
