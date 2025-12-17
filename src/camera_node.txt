#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "MvCameraControl.h"

// 图像回调函数类型
typedef void (*ImageCallbackFunc)(const cv::Mat& image, void* user_data);

class CameraNode {
 public:
  CameraNode(ImageCallbackFunc callback, void* user_data,
             const std::string& save_path = "/home/zzh/Pictures/hik/bmp")
      : callback_(callback), user_data_(user_data) {
    // 默认配置
    capture_fps_ = 30;  // 默认24fps

    // 初始化保存路径
    setSavePath(save_path);

    // 计算帧间隔（毫秒）
    frame_interval_ms_ = (capture_fps_ > 0) ? (1000 / capture_fps_) : 0;
    last_frame_time_ = std::chrono::system_clock::now();

    // 初始化海康相机
    initializeCamera();
  }

  ~CameraNode() {
    // 停止图像采集
    stopGrabbing();

    // 关闭相机
    closeCamera();

    // 反初始化SDK
    MV_CC_Finalize();
  }

  void start() {
    // 启动图像采集
    startGrabbing();
  }

  void setSavePath(const std::string& base_path) {
    // 创建基于当前时间的文件夹名
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    std::ostringstream folder_name;
    folder_name << std::put_time(&now_tm, "%Y%m%d_%H%M%S");

    // 完整的保存路径
    image_save_path_ = base_path + "/" + folder_name.str();

    // 创建目录
    try {
      std::filesystem::create_directories(image_save_path_);
      std::cout << "Image save path: " << image_save_path_ << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
      std::cerr << "Failed to create image save directory: " << e.what()
                << std::endl;
    }
  }

 private:
  void initializeCamera() {
    int nRet = MV_CC_Initialize();
    if (nRet != MV_OK) {
      std::cerr << "Failed to initialize camera SDK." << std::endl;
      return;
    }

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
      std::cerr << "No devices found." << std::endl;
      return;
    }

    MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[0]);

    // 尝试打开相机，先使用独占模式
    nRet = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);

    // 如果打开失败，可能是相机已被占用或其他原因
    int retry_count = 0;
    const int max_retries = 3;
    while (nRet != MV_OK && retry_count < max_retries) {
      retry_count++;
      std::cerr << "Failed to open camera: 0x" << std::hex << nRet
                << ". Retrying... (" << retry_count << "/" << max_retries << ")"
                << std::endl;
      nRet = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);
    }

    if (nRet != MV_OK) {
      std::cerr << "Failed to open camera after " << max_retries << " retries."
                << std::endl;
      return;
    }

    std::cout << "Camera opened successfully." << std::endl;

    // 设置ROI区域：宽度2400, 高度1600, 水平偏移60, 垂直偏移450
    unsigned int roi_width = 2400;
    unsigned int roi_height = 1600;
    unsigned int roi_offset_x = 60;
    unsigned int roi_offset_y = 450;

    // 设置ROI水平偏移
    nRet = MV_CC_SetIntValueEx(handle_, "OffsetX", roi_offset_x);
    if (nRet != MV_OK) {
      std::cerr << "Failed to set ROI OffsetX: 0x" << std::hex << nRet
                << std::endl;
    }

    // 设置ROI垂直偏移
    nRet = MV_CC_SetIntValueEx(handle_, "OffsetY", roi_offset_y);
    if (nRet != MV_OK) {
      std::cerr << "Failed to set ROI OffsetY: 0x" << std::hex << nRet
                << std::endl;
    }

    // 设置ROI宽度
    nRet = MV_CC_SetIntValueEx(handle_, "Width", roi_width);
    if (nRet != MV_OK) {
      std::cerr << "Failed to set ROI Width: 0x" << std::hex << nRet
                << std::endl;
    }

    // 设置ROI高度
    nRet = MV_CC_SetIntValueEx(handle_, "Height", roi_height);
    if (nRet != MV_OK) {
      std::cerr << "Failed to set ROI Height: 0x" << std::hex << nRet
                << std::endl;
    }

    // 关闭触发模式
    nRet =
        MV_CC_SetEnumValue(handle_, "TriggerMode", 0);  // Set trigger mode off
    if (nRet != MV_OK) {
      std::cerr << "Failed to set trigger mode: 0x" << std::hex << nRet
                << std::endl;
      return;
    }

    // 设置帧率为24fps
    MVCC_FLOATVALUE stFrameRateValue = {0};
    nRet =
        MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &stFrameRateValue);
    if (nRet == MV_OK) {
      std::cout << "Current FrameRate: " << stFrameRateValue.fCurValue
                << " Hz, Range: [" << stFrameRateValue.fMin << ", "
                << stFrameRateValue.fMax << "] Hz" << std::endl;

      float target_frame_rate = 24.0f;

      // 确保目标帧率在有效范围内
      if (target_frame_rate < stFrameRateValue.fMin) {
        target_frame_rate = stFrameRateValue.fMin;
        std::cerr << "Target frame rate 24.0 Hz is below minimum, using "
                  << target_frame_rate << " Hz" << std::endl;
      }

      if (target_frame_rate > stFrameRateValue.fMax) {
        target_frame_rate = stFrameRateValue.fMax;
        std::cerr << "Target frame rate 24.0 Hz is above maximum, using "
                  << target_frame_rate << " Hz" << std::endl;
      }

      // 设置帧率
      nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
                                 target_frame_rate);
      if (nRet == MV_OK) {
        std::cout << "AcquisitionFrameRate set to: " << target_frame_rate
                  << " Hz" << std::endl;
      } else {
        std::cerr << "Failed to set frame rate: 0x" << std::hex << nRet
                  << std::endl;
      }
    }

    // 关闭自动曝光
    nRet = MV_CC_SetEnumValue(handle_, "ExposureAuto", 1);  // 1 means off
    if (nRet == MV_OK) {
      std::cout << "Auto Exposure disabled" << std::endl;

      // 设置更短的曝光时间以提高帧率
      // 获取当前曝光时间范围
      MVCC_FLOATVALUE stExposureTimeValue = {0};
      nRet = MV_CC_GetFloatValue(handle_, "ExposureTime", &stExposureTimeValue);
      if (nRet == MV_OK) {
        // 设置最短曝光时间以获得最高帧率
        nRet = MV_CC_SetFloatValue(handle_, "ExposureTime",
                                   stExposureTimeValue.fMin);
        if (nRet == MV_OK) {
          std::cout << "ExposureTime set to: " << stExposureTimeValue.fMin
                    << " us for higher FPS" << std::endl;
        }
      }
    } else {
      std::cerr << "Failed to disable auto exposure: 0x" << std::hex << nRet
                << std::endl;
    }
  }

  void startGrabbing() {
    int nRet = MV_CC_StartGrabbing(handle_);
    if (nRet != MV_OK) {
      std::cerr << "Failed to start grabbing: 0x" << std::hex << nRet
                << std::endl;
      return;
    }

    // 获取PayloadSize
    uint64_t nPayloadSize = 0;
    unsigned int nAlignment = 0;
    nRet = MV_CC_GetPayloadSize(handle_, &nPayloadSize, &nAlignment);
    if (nRet != MV_OK) {
      std::cerr << "Failed to get payload size: 0x" << std::hex << nRet
                << std::endl;
      return;
    }

    // 分配缓冲区
    unsigned char* pData = new unsigned char[nPayloadSize];
    if (pData == nullptr) {
      std::cerr << "Failed to allocate buffer." << std::endl;
      return;
    }

    // 直接采集循环
    int frame_count = 0;
    int saved_frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
      MV_FRAME_OUT_INFO_EX stFrameInfo = {0};

      // 获取一帧图像
      nRet = MV_CC_GetOneFrameTimeout(
          handle_, pData, (unsigned int)nPayloadSize, &stFrameInfo, 1000);

      if (nRet == MV_OK) {
        // Prepare output buffer for conversion (estimate)
        unsigned int nWidth = stFrameInfo.nWidth;
        unsigned int nHeight = stFrameInfo.nHeight;
        unsigned int nFrameLen = stFrameInfo.nFrameLen;
        unsigned int nOutputBufferSize = nWidth * nHeight * 4 + 1024;
        unsigned char* pOutputBuffer = new unsigned char[nOutputBufferSize];

        MV_SAVE_IMAGE_PARAM_EX3 stSaveParam;
        memset(&stSaveParam, 0, sizeof(stSaveParam));
        stSaveParam.pData = pData;
        stSaveParam.nDataLen = nFrameLen;
        stSaveParam.enPixelType = stFrameInfo.enPixelType;
        stSaveParam.nWidth = nWidth;
        stSaveParam.nHeight = nHeight;
        stSaveParam.pImageBuffer = pOutputBuffer;
        stSaveParam.nBufferSize = nOutputBufferSize;
        // Use BMP format for raw image saving
        stSaveParam.enImageType = MV_Image_Bmp;
        // BMP doesn't use JPG quality parameter, so comment it out
        // stSaveParam.nJpgQuality = 95;
        stSaveParam.iMethodValue = 2;

        int nRet = MV_CC_SaveImageEx3(handle_, &stSaveParam);
        if (nRet != MV_OK || stSaveParam.nImageLen == 0) {
          // Conversion failed; cleanup and continue
          delete[] pOutputBuffer;
          continue;
        }

        // 直接保存SDK转换后的BMP数据

        std::string filename =
            image_save_path_ + "/" + std::to_string(saved_frame_count) + ".bmp";

        // 直接写入SDK转换后的BMP数据
        std::ofstream outfile(filename, std::ios::binary);
        if (outfile.is_open()) {
          outfile.write(reinterpret_cast<const char*>(stSaveParam.pImageBuffer),
                        stSaveParam.nImageLen);
          outfile.close();
        }

        // 更新帧率统计
        frame_count++;
        saved_frame_count++;
        auto now_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           now_time - start_time)
                           .count();

        if (elapsed >= 1) {
          double fps = static_cast<double>(frame_count) / elapsed;
          std::cout << "Real-time FPS: " << fps << std::endl;
          frame_count = 0;
          start_time = now_time;
        }

        // 如果需要回调，可以解码，但建议移除或优化
        // if (callback_) {
        //   // 只在需要回调时才解码
        //   std::vector<unsigned char> imgBuf(
        //       stSaveParam.pImageBuffer,
        //       stSaveParam.pImageBuffer + stSaveParam.nImageLen);
        //   cv::Mat img = cv::imdecode(imgBuf, cv::IMREAD_COLOR);

        //   if (!img.empty()) {
        //     callback_(img, user_data_);
        //   }
        // }

        // 释放SDK转换后的数据
        delete[] pOutputBuffer;
      } else {
        // 如果获取失败，等待一段时间再重试
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    // 释放缓冲区
    if (pData != nullptr) {
      delete[] pData;
      pData = nullptr;
    }
  }

  void stopGrabbing() {
    int nRet = MV_CC_StopGrabbing(handle_);
    if (nRet != MV_OK) {
      std::cerr << "Failed to stop grabbing." << std::endl;
    }
  }

  void closeCamera() {
    int nRet = MV_CC_CloseDevice(handle_);
    if (nRet != MV_OK) {
      std::cerr << "Failed to close camera." << std::endl;
    }

    MV_CC_DestroyHandle(handle_);
  }

  ImageCallbackFunc callback_;
  void* user_data_;
  void* handle_ = nullptr;
  int capture_fps_;
  int frame_interval_ms_;
  std::chrono::system_clock::time_point last_frame_time_;
  std::string image_save_path_;  // 图像保存路径
};

// 示例回调函数
void exampleImageCallback(const cv::Mat& image, void* user_data) {
  // 在这里处理图像，例如显示或保存
  // cv::imshow("Camera Image", image);
  // cv::waitKey(1);  // 必须添加这个，否则窗口无法显示
}

int main(int argc, char** argv) {
  // 创建相机节点并传入回调函数
  CameraNode camera(exampleImageCallback, nullptr);

  // 启动采集
  camera.start();

  return 0;
}
