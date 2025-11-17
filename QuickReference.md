# MVS SDK 快速参考卡片

## 1. 核心 API 速查表

### 初始化与清理
```c
MV_CC_Initialize()                   // SDK 初始化（必须首先调用）
MV_CC_Finalize()                     // SDK 清理（最后调用）
```

### 设备管理
```c
MV_CC_EnumDevices(nDeviceType, &stDeviceList)
MV_CC_EnumDevicesEx(nDeviceType, &stDeviceList, szManufacturerName)
MV_CC_CreateHandle(&handle, pDevInfo)
MV_CC_DestroyHandle(handle)
MV_CC_OpenDevice(handle, nAccessMode, nSwitchoverKey)
MV_CC_CloseDevice(handle)
```

### 采集控制
```c
MV_CC_StartGrabbing(handle)
MV_CC_StopGrabbing(handle)
MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stFrameInfo, nTimeoutMs)
MV_CC_GetImageBuffer(handle, &stOutFrame, timeoutMs)
MV_CC_FreeImageBuffer(handle, &stOutFrame)
```

### 参数设置（通用节点 API）
```c
// 读取参数
MV_CC_GetIntValueEx(handle, "ParameterName", &stValue)
MV_CC_GetFloatValue(handle, "ParameterName", &stValue)
MV_CC_GetEnumValue(handle, "ParameterName", &stValue)
MV_CC_GetBoolValue(handle, "ParameterName", &nValue)
MV_CC_GetStringValue(handle, "ParameterName", szValue)

// 写入参数
MV_CC_SetIntValueEx(handle, "ParameterName", &stValue)
MV_CC_SetFloatValue(handle, "ParameterName", fValue)
MV_CC_SetEnumValue(handle, "ParameterName", nValue)
MV_CC_SetEnumValueByString(handle, "ParameterName", szValue)
MV_CC_SetBoolValue(handle, "ParameterName", nValue)
MV_CC_SetStringValue(handle, "ParameterName", szValue)
```

### 高级功能
```c
MV_CC_GetPayloadSize(handle, &nPayloadSize, &nAlignment)
MV_CC_RegisterImageCallBackEx(handle, fnCallBack, pUserParam)
MV_CC_RegisterImageCallBackEx2(handle, fnCallBack, pUserParam)
MV_CC_ConvertPixelTypeEx(handle, &stConvertParam)
MV_CC_SaveImageEx3(handle, szImagePath, pImageBuffer, nImageBufferSize, ...)
```

---

## 2. 设备类型与接口常量

### 设备类型 (nTLayerType)
```c
#define MV_GIGE_DEVICE              0x00000001  // GigE Vision
#define MV_USB_DEVICE               0x00000002  // USB3 Vision
#define MV_CAMERALINK_DEVICE        0x00000004  // Camera Link
#define MV_VIR_GIGE_DEVICE          0x00000081  // 虚拟 GigE
#define MV_VIR_USB_DEVICE           0x00000082  // 虚拟 USB3
#define MV_GENTL_GIGE_DEVICE        0x00010001  // GenTL GigE
#define MV_GENTL_USB_DEVICE         0x00010002  // GenTL USB
#define MV_GENTL_CAMERALINK_DEVICE  0x00010004  // GenTL CameraLink
#define MV_GENTL_CXP_DEVICE         0x00010040  // GenTL CoaXPress
#define MV_GENTL_XOF_DEVICE         0x00010080  // GenTL XoF
```

### 访问模式 (nAccessMode)
```c
#define MV_ACCESS_Exclusive  0      // 独占访问
#define MV_ACCESS_Control    1      // 控制权限
#define MV_ACCESS_Monitor    2      // 监控权限（只读）
```

### 返回码
```c
#define MV_OK                           0x00000000  // 成功
#define MV_E_HANDLE                     0x80000001  // 错误的句柄
#define MV_E_SUPPORT                    0x80000002  // 功能不支持
#define MV_E_BUFOVER                    0x80000003  // 缓冲溢出
#define MV_E_CALLORDER                  0x80000004  // 调用顺序错误
#define MV_E_PARAMETER                  0x80000005  // 参数错误
#define MV_E_RESOURCE                   0x80000006  // 资源不足
#define MV_E_INVALIDATE                 0x80000007  // 设备未初始化
#define MV_E_NOTIMPLEMENTED             0x80000008  // 功能未实现
#define MV_E_NOTFOUND                   0x80000009  // 设备未发现
#define MV_E_BADSEQUENCE                0x8000000A  // 调用序列错误
#define MV_E_BADBUFFERSIZE              0x8000000B  // 缓冲区大小不足
#define MV_CC_IMAGE_NOT_VALID_ID        0xC0000001  // 图像无效（未就绪）
#define MV_CC_WAIT_TIMEOUT_ID           0xC0000002  // 获取图像超时
```

---

## 3. 数据结构速查

### 设备信息 (MV_CC_DEVICE_INFO)
```c
typedef struct {
    unsigned int nTLayerType;           // 设备类型
    
    union {
        MV_GIGE_DEVICE_INFO stGigEInfo;        // GigE 设备信息
        MV_USB3_DEVICE_INFO stUsb3VInfo;       // USB3 设备信息
        MV_CAMERALINK_DEVICE_INFO stCameraLinkInfo;  // CameraLink 信息
        // ... 其他类型
    } SpecialInfo;
} MV_CC_DEVICE_INFO;
```

### 帧输出信息 (MV_FRAME_OUT_INFO_EX)
```c
typedef struct {
    unsigned int nWidth;                // 图像宽度（像素）
    unsigned int nHeight;               // 图像高度（像素）
    enum MvGvspPixelType enPixelType;   // 像素格式
    unsigned int nFrameNum;             // 帧号
    unsigned int nSecondCount;          // 时间戳秒
    unsigned int nCycleCount;           // 时间戳周期计数
    unsigned int nCycleOffset;          // 时间戳偏移
    unsigned int nFrameLen;             // 实际帧数据长度（字节）
    unsigned int nWidth2;               // 保留，通常与 nWidth 相同
    unsigned int nHeight2;              // 保留，通常与 nHeight 相同
    unsigned int nReserved[4];          // 保留字段
    double fGain;                       // 增益值
    double fExposureTime;               // 曝光时间（微秒）
    unsigned int nAveGrayValue;         // 平均灰度值
    unsigned int nROIOffsetX;           // ROI 水平偏移
    unsigned int nROIOffsetY;           // ROI 垂直偏移
    unsigned short nFrameID;            // 帧 ID
    unsigned short nErrorCode;          // 错误码
    // ... 更多字段
} MV_FRAME_OUT_INFO_EX;
```

### 整数参数值 (MVCC_INTVALUE_EX)
```c
typedef struct {
    int64_t nCurValue;      // 当前值
    int64_t nMin;           // 最小值
    int64_t nMax;           // 最大值
    int64_t nInc;           // 步长
} MVCC_INTVALUE_EX;
```

### 浮点参数值 (MVCC_FLOATVALUE)
```c
typedef struct {
    float fCurValue;        // 当前值
    float fMin;             // 最小值
    float fMax;             // 最大值
} MVCC_FLOATVALUE;
```

### 枚举参数值 (MVCC_ENUMVALUE)
```c
typedef struct {
    unsigned int nCurValue;             // 当前值
    unsigned int nEnumEntryNums;        // 可用值个数
    MVCC_ENUM_ENTRY_EX *pEnumEntry;   // 枚举项数组
} MVCC_ENUMVALUE;
```

---

## 4. 像素格式常用枚举

### 黑白格式
```c
PixelType_Gvsp_Mono8           // Mono8（8 位）
PixelType_Gvsp_Mono10          // Mono10（10 位紧凑）
PixelType_Gvsp_Mono12          // Mono12（12 位紧凑）
PixelType_Gvsp_Mono14          // Mono14（14 位紧凑）
PixelType_Gvsp_Mono16          // Mono16（16 位）
PixelType_Gvsp_Mono32          // Mono32（32 位）
```

### Bayer 格式（彩色）
```c
PixelType_Gvsp_BayerGR8        // Bayer GR 8 位
PixelType_Gvsp_BayerRG8        // Bayer RG 8 位
PixelType_Gvsp_BayerGB8        // Bayer GB 8 位
PixelType_Gvsp_BayerBG8        // Bayer BG 8 位
PixelType_Gvsp_BayerGR10       // Bayer GR 10 位（紧凑）
PixelType_Gvsp_BayerGR12       // Bayer GR 12 位（紧凑）
PixelType_Gvsp_BayerGR16       // Bayer GR 16 位
```

### RGB/BGR 格式
```c
PixelType_Gvsp_RGB8_Packed     // RGB 8 位打包（3 字节/像素）
PixelType_Gvsp_BGR8_Packed     // BGR 8 位打包（3 字节/像素）
PixelType_Gvsp_RGBA8_Packed    // RGBA 8 位打包（4 字节/像素）
PixelType_Gvsp_BGRA8_Packed    // BGRA 8 位打包（4 字节/像素）
PixelType_Gvsp_RGB10_Packed    // RGB 10 位打包
PixelType_Gvsp_BGR10_Packed    // BGR 10 位打包
PixelType_Gvsp_RGB12_Packed    // RGB 12 位打包
PixelType_Gvsp_BGR12_Packed    // BGR 12 位打包
PixelType_Gvsp_RGB16_Packed    // RGB 16 位打包（6 字节/像素）
PixelType_Gvsp_BGR16_Packed    // BGR 16 位打包（6 字节/像素）
```

### YUV 格式
```c
PixelType_Gvsp_YUV411_Packed   // YUV 411 打包
PixelType_Gvsp_YUV422_YUYV     // YUV 422 YUYV（2 字节/像素）
PixelType_Gvsp_YUV422_UYVY     // YUV 422 UYVY（2 字节/像素）
PixelType_Gvsp_YUV444_Packed   // YUV 444 打包
PixelType_Gvsp_YCBCR411_8      // YCbCr 411 8 位
PixelType_Gvsp_YCBCR422_8      // YCbCr 422 8 位
PixelType_Gvsp_YCBCR444_8      // YCbCr 444 8 位
PixelType_Gvsp_YUV420SP        // YUV 420 SP（NV12，1.5 字节/像素）
```

### 其他格式
```c
PixelType_Gvsp_JPEG            // JPEG 压缩
PixelType_Gvsp_Coord3D_ABC32f  // 3D 坐标 ABC 32 位浮点
PixelType_Gvsp_Coord3D_AC32f   // 3D 坐标 AC 32 位浮点
PixelType_Gvsp_HB_Mono8        // 无损压缩 Mono8
PixelType_Gvsp_HB_BayerGR8     // 无损压缩 Bayer GR8
```

---

## 5. 常见参数名称（通用节点 API）

### 基本图像参数
```
"Width"                    // 图像宽度（像素）
"Height"                   // 图像高度（像素）
"OffsetX"                  // ROI 水平偏移
"OffsetY"                  // ROI 垂直偏移
"PixelFormat"              // 像素格式
"ReverseX"                 // 水平翻转
"ReverseY"                 // 垂直翻转
"Rotation"                 // 旋转角度
```

### 曝光与增益
```
"ExposureAuto"             // 自动曝光模式
"ExposureMode"             // 曝光模式（连续/脉冲）
"ExposureTime"             // 曝光时间（微秒）
"Gain"                     // 增益（dB）
"GainAuto"                 // 自动增益
"BlackLevel"               // 黑电平
"WhiteBalance"             // 白平衡
"BalanceRatioRed"          // 红色通道平衡
"BalanceRatioBlue"         // 蓝色通道平衡
```

### 帧率与采集
```
"AcquisitionFrameRate"     // 采集帧率（Hz）
"TriggerMode"              // 触发模式（0=关闭，1=启用）
"TriggerSource"            // 触发源（0=软件，1=Line0 等）
"TriggerDelay"             // 触发延迟（微秒）
"GrabStrategy"             // 采集策略
"PayloadSize"              // 帧数据大小（字节）
```

### GigE 网络参数
```
"GevIPAddress"             // GigE 设备 IP 地址
"GevSubnetMask"            // 子网掩码
"GevGateway"               // 网关
"IPConfigurationMode"      // IP 配置方式（静态/DHCP/LLA）
"GevSCPSPacketSize"        // 网络包大小
"GevSCPSInterPacketDelay"  // 包间延迟
"StreamBufferHandlingMode" // 流缓冲处理模式
```

---

## 6. 最小化代码骨架

### 同步拉取模式（推荐）
```cpp
#include "MvCameraControl.h"

int main() {
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
    unsigned char* pData = NULL;
    
    // 初始化 & 枚举
    MV_CC_Initialize();
    MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    
    if (stDeviceList.nDeviceNum == 0) return -1;
    
    // 创建句柄 & 打开设备
    MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
    
    // 分配缓冲
    uint64_t nPayloadSize = 0;
    unsigned int nAlignment = 0;
    MV_CC_GetPayloadSize(handle, &nPayloadSize, &nAlignment);
    pData = new unsigned char[nPayloadSize];
    
    // 启动采集
    MV_CC_StartGrabbing(handle);
    
    // 循环获取帧
    for (int i = 0; i < 10; ++i) {
        MV_FRAME_OUT_INFO_EX stFrameInfo = {0};
        int nRet = MV_CC_GetOneFrameTimeout(
            handle, pData, (unsigned int)nPayloadSize, &stFrameInfo, 1000
        );
        if (nRet == MV_OK) {
            // 处理图像...
            printf("Frame: %u x %u\n", stFrameInfo.nWidth, stFrameInfo.nHeight);
        }
    }
    
    // 清理
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    delete[] pData;
    
    return 0;
}
```

### 异步回调模式（高级）
```cpp
void __stdcall ImageCallbackFunc(
    unsigned char* pData,
    MV_FRAME_OUT_INFO_EX* pFrameInfo,
    void* pUser
) {
    if (pFrameInfo) {
        printf("Callback: Frame %u, %u x %u\n",
               pFrameInfo->nFrameNum,
               pFrameInfo->nWidth,
               pFrameInfo->nHeight);
        // 处理图像...
    }
}

int main() {
    // ... 初始化代码 ...
    
    // 注册回调
    MV_CC_RegisterImageCallBackEx(handle, ImageCallbackFunc, NULL);
    
    // 启动采集（回调由 SDK 内部线程调用）
    MV_CC_StartGrabbing(handle);
    
    sleep(10);  // 让 SDK 自动调用回调 10 秒
    
    MV_CC_StopGrabbing(handle);
    
    // ... 清理代码 ...
}
```

---

## 7. 编译与链接

### GCC/G++ 编译
```bash
g++ -c MyProgram.cpp -I/opt/MVS/include
g++ MyProgram.o -o MyProgram -L/opt/MVS/lib/64 -lMvCameraControl -lrt
```

### CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.10)
project(MVSCapture)

set(CMAKE_CXX_STANDARD 11)
set(MVS_SDK_PATH /opt/MVS)

include_directories(${MVS_SDK_PATH}/include)
link_directories(${MVS_SDK_PATH}/lib/64)

add_executable(SimpleCapture SimpleCapture.cpp)
target_link_libraries(SimpleCapture MvCameraControl rt)
```

### Makefile（简化版）
```makefile
CXX = g++
CXXFLAGS = -I/opt/MVS/include -std=c++11
LDFLAGS = -L/opt/MVS/lib/64 -lMvCameraControl -lrt

SimpleCapture: SimpleCapture.o
	$(CXX) SimpleCapture.o -o SimpleCapture $(LDFLAGS)

SimpleCapture.o: SimpleCapture.cpp
	$(CXX) $(CXXFLAGS) -c SimpleCapture.cpp

clean:
	rm -f *.o SimpleCapture
```

---

## 8. 错误处理模板

```cpp
#define CHECK_MV_OK(nRet, msg) \
    if (MV_OK != nRet) { \
        printf("[ERROR] %s: 0x%x\n", msg, nRet); \
        goto cleanup; \
    }

int main() {
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
    int nRet = MV_OK;
    
    // 初始化
    nRet = MV_CC_Initialize();
    CHECK_MV_OK(nRet, "Initialize");
    
    // 枚举
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    CHECK_MV_OK(nRet, "EnumDevices");
    
    if (stDeviceList.nDeviceNum == 0) {
        printf("[ERROR] No device found\n");
        goto cleanup;
    }
    
    // ... 其他操作 ...
    
cleanup:
    if (handle) {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
    }
    MV_CC_Finalize();
    
    return nRet;
}
```

---

## 9. 性能关键参数

| 参数 | 典型值 | 调整建议 |
|------|--------|---------|
| `GetOneFrameTimeout` 超时值 | 1000 ms | 根据帧率调整：`1000 / FrameRate` |
| 缓冲大小 | `GetPayloadSize()` | 必须 ≥ 实际需要 |
| GigE 包大小 | 1500 字节（默认） | 可设为 MTU 值（通常 9000） |
| USB 带宽 | 自动协商 | 检查 USB 3.0 连接 |
| 采集线程数 | 1（默认） | 高速度使用异步回调 |

---

## 10. 快速故障排查清单

- [ ] SDK 已安装到 `/opt/MVS`
- [ ] 编译器支持 C++11 或更高版本
- [ ] 库路径正确：`-L/opt/MVS/lib/64` 或 `lib/32`
- [ ] 头文件路径正确：`-I/opt/MVS/include`
- [ ] 链接库正确：`-lMvCameraControl -lrt`
- [ ] 环境变量设置：`export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH`
- [ ] 相机已连接且识别为设备
- [ ] 相机已开启
- [ ] 网络/USB 连接正常
- [ ] 尝试使用 root 权限运行

---

**快速参考卡片版本**：1.0
**MVS SDK 版本**：4.6.0+
**最后更新**：2024 年
