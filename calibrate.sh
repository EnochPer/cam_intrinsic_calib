#!/bin/bash

# ============================================================================
# 相机标定快速启动脚本
# ============================================================================
#
# 使用方法：
#   chmod +x calibrate.sh
#   ./calibrate.sh /path/to/images [output.yaml] [square_size]
#
# 示例：
#   ./calibrate.sh ~/Pictures/hik
#   ./src/cam_intrinsic_calib//calibrate.sh /home/zzh/hikon_cam/hik camera_calib.yaml 45
#
# ============================================================================

set -e

# 配置变量
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="/home/zzh/hikon_cam"
IMAGE_DIR="${1:-.}"
OUTPUT_FILE="${2:-camera_calibration.yaml}"
SQUARE_SIZE="${3:-20}"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============================================================================
# 函数定义
# ============================================================================

print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════╗"
    echo "║   相机内参标定快速启动工具 v1.0        ║"
    echo "╚════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_step() {
    echo -e "${BLUE}[步骤]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[成功]${NC} $1"
}

print_error() {
    echo -e "${RED}[错误]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[警告]${NC} $1"
}

check_directory() {
    if [ ! -d "$IMAGE_DIR" ]; then
        print_error "图像目录不存在: $IMAGE_DIR"
        exit 1
    fi
    
    # 检查是否包含图像文件
    local image_count=$(find "$IMAGE_DIR" -maxdepth 1 \
        \( -iname "*.bmp" -o -iname "*.jpg" -o -iname "*.jpeg" -o -iname "*.png" \) \
        | wc -l)
    
    if [ $image_count -eq 0 ]; then
        print_error "目录中未找到任何图像文件 (BMP/JPG/PNG): $IMAGE_DIR"
        exit 1
    fi
    
    echo "  找到 $image_count 张图像文件"
}

build_project() {
    print_step "编译项目..."
    
    cd "$WORKSPACE_ROOT"

    
    if ! colcon build --packages-select cam_intrinsic_calib 2>&1 | tail -5; then
        print_error "编译失败"
        exit 1
    fi
    
    print_success "编译完成"
}

run_calibration() {
    print_step "运行标定程序..."
    echo "  输入目录: $IMAGE_DIR"
    echo "  输出文件: $OUTPUT_FILE"
    echo "  方块边长: ${SQUARE_SIZE}mm"
    echo ""
    
    # 设置环境
    source "$WORKSPACE_ROOT/install/setup.bash" 2>/dev/null || true
    
    # 查找可执行文件
    local calibrate_exe="/home/zzh/hikon_cam/install/cam_intrinsic_calib/lib/cam_intrinsic_calib/calibrate_camera"
    
    if [ -z "$calibrate_exe" ]; then
        print_error "找不到 calibrate_camera 可执行文件"
        echo "  请确保已编译项目: colcon build --packages-select cam_intrinsic_calib"
        exit 1
    fi
    
    # 运行标定
    "$calibrate_exe" "$IMAGE_DIR" --output "$OUTPUT_FILE" --square-size "$SQUARE_SIZE" --display
    
    if [ $? -eq 0 ]; then
        print_success "标定完成"
        return 0
    else
        print_error "标定失败"
        return 1
    fi
}

check_result() {
    print_step "检查结果文件..."
    
    if [ -f "$OUTPUT_FILE" ]; then
        print_success "标定文件已生成: $OUTPUT_FILE"
        echo "  文件大小: $(du -h "$OUTPUT_FILE" | cut -f1)"
        
        # 检查文本结果文件
        local text_file="${OUTPUT_FILE%.yaml}.txt"
        if [ -f "$text_file" ]; then
            print_success "文本结果文件: $text_file"
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            head -20 "$text_file"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
        fi
    else
        print_error "标定文件未找到: $OUTPUT_FILE"
        return 1
    fi
}

# ============================================================================
# 主程序
# ============================================================================

main() {
    print_header
    
    echo ""
    print_step "验证输入参数..."
    echo "  图像目录: $IMAGE_DIR"
    echo "  输出文件: $OUTPUT_FILE"
    echo "  方块大小: ${SQUARE_SIZE}mm"
    echo ""
    
    check_directory
    echo ""
    
    print_step "验证编译状态..."
    if [ ! -d "$WORKSPACE_ROOT/install" ]; then
        print_warning "install 目录不存在，需要编译"
        echo ""
        build_project
        echo ""
    else
        print_success "已编译"
        echo ""
    fi
    
    run_calibration || exit 1
    echo ""
    
    check_result
    
    echo -e "${GREEN}"
    echo "╔════════════════════════════════════════╗"
    echo "║      标定流程完成！                    ║"
    echo "╚════════════════════════════════════════╝"
    echo -e "${NC}"
    
    echo ""
    echo "后续步骤:"
    echo "  1. 检查输出文件: $OUTPUT_FILE"
    echo "  2. 验证重投影误差是否在可接受范围内"
    echo "  3. 在您的应用中使用标定结果"
    echo ""
    
    if [ -f "CALIBRATION_GUIDE.md" ]; then
        echo "详细文档: CALIBRATION_GUIDE.md"
        echo ""
    fi
}

# 执行主程序
main "$@"
