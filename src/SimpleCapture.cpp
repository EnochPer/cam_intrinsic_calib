#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "MvCameraControl.h"
#include "CameraParams.h"
#include "PixelType.h"

/**
 * @file SimpleCapture.cpp
 * @brief 最小化 C++ 图像采集示例程序
 * @details
 *   - 演示如何用 MVS SDK 以同步（拉取）方式采集一帧或多帧图像
 *   - 使用 MV_CC_GetOneFrameTimeout 方式获取图像
 *   - 包含了设备枚举、参数设置、采集启停等完整流程
 *   - 支持 GigE、USB3、CameraLink 等多种接口（根据设备类型自动检测）
 *
 * 编译：
 *   # 进入此目录
 *   cd /opt/MVS/Samples/64/C++/SimpleCaptureDemo
 *   # 使用 make 编译
 *   make
 *
 * 运行：
 *   # 需要连接至少一台相机
 *   ./SimpleCapture
 *   或指定采集帧数（默认 5）：
 *   ./SimpleCapture 10
 */

// ============================================================================
// 辅助函数：打印错误消息
// ============================================================================
void PrintErrorCode(int nRet) {
    if (nRet == MV_OK) {
        printf("[OK] 成功\n");
    } else {
        printf("[ERROR] 错误码: 0x%08x\n", nRet);
    }
}

// ============================================================================
// 辅助函数：打印设备信息
// ============================================================================
void PrintDeviceInfo(MV_CC_DEVICE_INFO* pDevInfo) {
    if (!pDevInfo) {
        printf("[ERROR] 设备信息指针为空\n");
        return;
    }

    printf("========== 设备信息 ==========\n");
    printf("设备类型: 0x%x\n", pDevInfo->nTLayerType);

    if (pDevInfo->nTLayerType == MV_GIGE_DEVICE) {
        printf("  接口类型: GigE\n");
        printf("  制造商: %s\n", pDevInfo->SpecialInfo.stGigEInfo.chManufacturerName);
        printf("  型号: %s\n", pDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("  序列号: %s\n", pDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        printf("  IP 地址: %d.%d.%d.%d\n",
               (pDevInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 24) & 0xFF,
               (pDevInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 16) & 0xFF,
               (pDevInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 8) & 0xFF,
               pDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xFF);
    } else if (pDevInfo->nTLayerType == MV_USB_DEVICE) {
        printf("  接口类型: USB3 Vision\n");
        printf("  制造商: %s\n", pDevInfo->SpecialInfo.stUsb3VInfo.chManufacturerName);
        printf("  型号: %s\n", pDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("  序列号: %s\n", pDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
    } else {
        printf("  接口类型: 其他 (0x%x)\n", pDevInfo->nTLayerType);
    }
    printf("==============================\n\n");
}

// ============================================================================
// 主函数：图像采集演示
// ============================================================================
int main(int argc, char* argv[]) {
    int nRet = MV_OK;
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
    int nFrameCount = 5;  // 默认采集 5 帧

    // 解析命令行参数（可选）
    if (argc > 1) {
        nFrameCount = atoi(argv[1]);
        if (nFrameCount <= 0 || nFrameCount > 1000) {
            printf("[WARN] 帧数参数无效，使用默认值 5\n");
            nFrameCount = 5;
        }
    }

    printf("========================================\n");
    printf("  MVS SDK 最小化图像采集示例\n");
    printf("  计划采集 %d 帧\n", nFrameCount);
    printf("========================================\n\n");

    // ========================================================================
    // 第 1 步：初始化 SDK
    // ========================================================================
    printf("[步骤 1] SDK 初始化...\n");
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        printf("[ERROR] SDK 初始化失败: ");
        PrintErrorCode(nRet);
        return -1;
    }
    printf("[OK] SDK 初始化成功\n\n");

    // ========================================================================
    // 第 2 步：枚举设备
    // ========================================================================
    printf("[步骤 2] 枚举设备...\n");
    // 参数说明：
    //   MV_GIGE_DEVICE | MV_USB_DEVICE：枚举 GigE 和 USB3 设备
    //   &stDeviceList：设备列表（SDK 内部分配内存）
    //   NULL：不过滤特定制造商
    unsigned int nDeviceType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_CAMERALINK_DEVICE;
    nRet = MV_CC_EnumDevices(nDeviceType, &stDeviceList);
    if (MV_OK != nRet) {
        printf("[ERROR] 设备枚举失败: ");
        PrintErrorCode(nRet);
        MV_CC_Finalize();
        return -1;
    }

    if (stDeviceList.nDeviceNum == 0) {
        printf("[ERROR] 未发现任何相机设备\n");
        printf("        请检查：\n");
        printf("        1. 相机是否连接且已开启\n");
        printf("        2. 网卡/USB 线缆连接是否正常\n");
        printf("        3. 相机驱动是否已安装\n");
        MV_CC_Finalize();
        return -1;
    }

    printf("[OK] 找到 %u 个设备\n\n", stDeviceList.nDeviceNum);

    // ========================================================================
    // 第 3 步：选择第一个设备并打印其信息
    // ========================================================================
    printf("[步骤 3] 选择设备并创建句柄...\n");
    MV_CC_DEVICE_INFO* pDevInfo = stDeviceList.pDeviceInfo[0];
    PrintDeviceInfo(pDevInfo);

    // ========================================================================
    // 第 4 步：创建设备句柄
    // ========================================================================
    printf("[步骤 4] 创建设备句柄...\n");
    // 参数说明：
    //   &handle：设备句柄指针（出参，由 SDK 填充）
    //   pDevInfo：设备信息（来自枚举结果）
    nRet = MV_CC_CreateHandle(&handle, pDevInfo);
    if (MV_OK != nRet) {
        printf("[ERROR] 创建句柄失败: ");
        PrintErrorCode(nRet);
        MV_CC_Finalize();
        return -1;
    }
    printf("[OK] 句柄创建成功\n\n");

    // ========================================================================
    // 第 5 步：打开设备
    // ========================================================================
    printf("[步骤 5] 打开设备...\n");
    // 参数说明：
    //   handle：设备句柄
    //   MV_ACCESS_Exclusive：独占访问权限（其他应用只能读）
    //   0：切换密钥（GigE 专属，一般设为 0）
    nRet = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
    if (MV_OK != nRet) {
        printf("[ERROR] 设备打开失败: ");
        PrintErrorCode(nRet);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return -1;
    }
    printf("[OK] 设备打开成功\n\n");

    // ========================================================================
    // 第 6 步：获取有效 Payload 大小，用于分配接收缓冲
    // ========================================================================
    printf("[步骤 6] 查询 Payload 大小...\n");
    uint64_t nPayloadSize = 0;
    unsigned int nAlignment = 0;
    // 参数说明：
    //   handle：设备句柄
    //   &nPayloadSize：返回一帧数据所需的缓冲大小（含图像 + Chunk 数据）
    //   &nAlignment：对齐字节数（用于优化内存访问，一般为 2^n）
    nRet = MV_CC_GetPayloadSize(handle, &nPayloadSize, &nAlignment);
    if (MV_OK != nRet) {
        printf("[WARN] 查询 Payload 大小失败: ");
        PrintErrorCode(nRet);
        // 降级处理：使用一个足够大的预设值
        nPayloadSize = 16 * 1024 * 1024;  // 16 MB
        printf("        使用默认缓冲大小: %lu 字节\n", nPayloadSize);
    } else {
        printf("[OK] Payload 大小: %lu 字节, 对齐: %u 字节\n\n", nPayloadSize, nAlignment);
    }

    // 分配接收缓冲
    unsigned char* pData = new unsigned char[nPayloadSize];
    if (!pData) {
        printf("[ERROR] 内存分配失败\n");
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return -1;
    }
    printf("[OK] 接收缓冲分配成功: %p\n\n", (void*)pData);

    // ========================================================================
    // 第 7 步：（可选）设置相机参数
    // ========================================================================
    printf("[步骤 7] 设置相机参数...\n");

    // 示例 1：获取当前宽度
    MVCC_INTVALUE_EX stWidth = {0};
    nRet = MV_CC_GetIntValueEx(handle, "Width", &stWidth);
    if (MV_OK == nRet) {
        printf("[INFO] 当前图像宽: %ld, 范围: [%ld, %ld]\n",
               stWidth.nCurValue, stWidth.nMin, stWidth.nMax);
    }

    // 示例 2：获取当前高度
    MVCC_INTVALUE_EX stHeight = {0};
    nRet = MV_CC_GetIntValueEx(handle, "Height", &stHeight);
    if (MV_OK == nRet) {
        printf("[INFO] 当前图像高: %ld, 范围: [%ld, %ld]\n",
               stHeight.nCurValue, stHeight.nMin, stHeight.nMax);
    }

    // 示例 3：查询像素格式
    MVCC_ENUMVALUE stPixelFormat = {0};
    nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &stPixelFormat);
    if (MV_OK == nRet) {
        printf("[INFO] 当前像素格式值: 0x%x\n", stPixelFormat.nCurValue);
    }

    // 示例 4：设置像素格式为 Mono8（黑白）（可选，如果相机支持）
    // 注：格式设置取决于相机是否支持，可能会失败（此时保持原设置）
    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_Mono8);
    if (MV_OK == nRet) {
        printf("[OK] 像素格式已设置为 Mono8\n");
    } else {
        printf("[INFO] 无法设置 Mono8 格式（相机可能不支持），保持原格式\n");
    }

    printf("\n");

    // ========================================================================
    // 第 8 步：启动采集
    // ========================================================================
    printf("[步骤 8] 启动图像采集...\n");
    // 参数说明：
    //   handle：设备句柄
    // 调用此函数后，相机开始发送图像数据。
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        printf("[ERROR] 启动采集失败: ");
        PrintErrorCode(nRet);
        delete[] pData;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return -1;
    }
    printf("[OK] 采集已启动\n\n");

    // ========================================================================
    // 第 9 步：循环获取图像
    // ========================================================================
    printf("[步骤 9] 开始获取图像...\n");
    printf("========================================\n");

    for (int i = 0; i < nFrameCount; ++i) {
        printf("[帧 %d] 等待图像...\n", i + 1);

        // 准备帧信息结构（用于接收图像的元数据）
        MV_FRAME_OUT_INFO_EX stFrameInfo = {0};

        // 获取一帧图像，带超时机制
        // 参数说明：
        //   handle：设备句柄
        //   pData：图像数据接收缓冲指针
        //   nPayloadSize：缓冲大小（字节）
        //   &stFrameInfo：帧信息结构（出参，包含宽、高、像素格式、时间戳等）
        //   1000：超时时间（毫秒，1000 = 1 秒）
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, (unsigned int)nPayloadSize, &stFrameInfo, 1000);
        if (MV_OK != nRet) {
            printf("[WARN] 获取图像失败或超时: 0x%08x\n", nRet);
            continue;
        }

        // 打印帧信息
        printf("  [OK] 图像接收成功\n");
        printf("      尺寸: %u x %u\n", stFrameInfo.nWidth, stFrameInfo.nHeight);
        printf("      像素格式: 0x%lx\n", stFrameInfo.enPixelType);
        printf("      帧号: %u\n", stFrameInfo.nFrameNum);
        printf("      时间戳: %u:%u:%u\n",
               stFrameInfo.nSecondCount,
               stFrameInfo.nCycleCount,
               stFrameInfo.nCycleOffset);
        printf("      数据长度: %u 字节\n", stFrameInfo.nFrameLen);
        printf("\n");

        // ====================================================================
        // 可选处理步骤：保存图像为高质量格式
        // ====================================================================
        // 使用 MV_CC_SaveImageEx3 保存为 BMP（无损）或高质量 JPEG 格式
        
        // 分配输出缓冲（预估最大可能的大小）
        unsigned int nOutputBufferSize = stFrameInfo.nWidth * stFrameInfo.nHeight * 4 + 1024;
        unsigned char* pOutputBuffer = new unsigned char[nOutputBufferSize];
        
        if (!pOutputBuffer) {
            printf("      [警告] 内存分配失败，跳过图像保存\n\n");
            continue;
        }
        
        // ====================================================================
        // 方式 1：保存为 BMP 格式（无损，推荐）
        // ====================================================================
        {
            MV_SAVE_IMAGE_PARAM_EX3 stSaveParam = {0};
            stSaveParam.pData = pData;
            stSaveParam.nDataLen = stFrameInfo.nFrameLen;
            stSaveParam.enPixelType = stFrameInfo.enPixelType;
            stSaveParam.nWidth = stFrameInfo.nWidth;
            stSaveParam.nHeight = stFrameInfo.nHeight;
            stSaveParam.pImageBuffer = pOutputBuffer;
            stSaveParam.nBufferSize = nOutputBufferSize;
            stSaveParam.enImageType = MV_Image_Bmp;  // BMP 格式：无损
            stSaveParam.iMethodValue = 2;  // 插值方法：2-最优（仅对 Bayer 格式有效）
            
            int nSaveRet = MV_CC_SaveImageEx3(handle, &stSaveParam);
            if (nSaveRet == MV_OK && stSaveParam.nImageLen > 0) {
                // 保存 BMP 文件
                char bmpFilename[256];
                snprintf(bmpFilename, sizeof(bmpFilename), "frame_%03d.bmp", i + 1);
                FILE* fpBmp = fopen(bmpFilename, "wb");
                if (fpBmp) {
                    fwrite(stSaveParam.pImageBuffer, 1, stSaveParam.nImageLen, fpBmp);
                    fclose(fpBmp);
                    printf("      [保存] %s (%u 字节, BMP 无损)\n", 
                           bmpFilename, stSaveParam.nImageLen);
                } else {
                    printf("      [警告] 无法打开 BMP 文件\n");
                }
            } else {
                printf("      [警告] BMP 转换失败 (0x%08x)\n", nSaveRet);
            }
        }
        
        // ====================================================================
        // 方式 2：保存为高质量 JPEG 格式（推荐用于高速传输）
        // ====================================================================
        {
            MV_SAVE_IMAGE_PARAM_EX3 stSaveParam = {0};
            stSaveParam.pData = pData;
            stSaveParam.nDataLen = stFrameInfo.nFrameLen;
            stSaveParam.enPixelType = stFrameInfo.enPixelType;
            stSaveParam.nWidth = stFrameInfo.nWidth;
            stSaveParam.nHeight = stFrameInfo.nHeight;
            stSaveParam.pImageBuffer = pOutputBuffer;
            stSaveParam.nBufferSize = nOutputBufferSize;
            stSaveParam.enImageType = MV_Image_Jpeg;  // JPEG 格式
            stSaveParam.nJpgQuality = 95;  // 质量等级：95（最高 99，最低 50）
            stSaveParam.iMethodValue = 2;  // 插值方法：2-最优
            
            int nSaveRet = MV_CC_SaveImageEx3(handle, &stSaveParam);
            if (nSaveRet == MV_OK && stSaveParam.nImageLen > 0) {
                // 保存 JPEG 文件
                char jpegFilename[256];
                snprintf(jpegFilename, sizeof(jpegFilename), "frame_%03d.jpg", i + 1);
                FILE* fpJpeg = fopen(jpegFilename, "wb");
                if (fpJpeg) {
                    fwrite(stSaveParam.pImageBuffer, 1, stSaveParam.nImageLen, fpJpeg);
                    fclose(fpJpeg);
                    printf("      [保存] %s (%u 字节, JPEG 质量 95)\n\n", 
                           jpegFilename, stSaveParam.nImageLen);
                } else {
                    printf("      [警告] 无法打开 JPEG 文件\n\n");
                }
            } else {
                printf("      [警告] JPEG 转换失败 (0x%08x)\n\n", nSaveRet);
            }
        }
        
        // 释放输出缓冲
        delete[] pOutputBuffer;
        pOutputBuffer = NULL;
    }

    printf("========================================\n");
    printf("[OK] 图像获取完毕\n\n");

    // ========================================================================
    // 第 10 步：停止采集
    // ========================================================================
    printf("[步骤 10] 停止采集...\n");
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
        printf("[ERROR] 停止采集失败: ");
        PrintErrorCode(nRet);
    } else {
        printf("[OK] 采集已停止\n\n");
    }

    // ========================================================================
    // 第 11 步：关闭设备
    // ========================================================================
    printf("[步骤 11] 关闭设备...\n");
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
        printf("[ERROR] 设备关闭失败: ");
        PrintErrorCode(nRet);
    } else {
        printf("[OK] 设备已关闭\n\n");
    }

    // ========================================================================
    // 第 12 步：销毁设备句柄
    // ========================================================================
    printf("[步骤 12] 销毁句柄...\n");
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
        printf("[ERROR] 句柄销毁失败: ");
        PrintErrorCode(nRet);
    } else {
        printf("[OK] 句柄已销毁\n\n");
    }

    // ========================================================================
    // 第 13 步：释放 SDK 资源
    // ========================================================================
    printf("[步骤 13] 释放 SDK...\n");
    nRet = MV_CC_Finalize();
    if (MV_OK != nRet) {
        printf("[ERROR] SDK 释放失败: ");
        PrintErrorCode(nRet);
    } else {
        printf("[OK] SDK 已释放\n\n");
    }

    // 释放缓冲内存
    delete[] pData;
    pData = NULL;

    printf("========================================\n");
    printf("  程序执行完毕\n");
    printf("========================================\n");

    return 0;
}
