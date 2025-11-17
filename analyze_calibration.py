#!/usr/bin/env python3
"""
相机标定结果分析工具
用于读取和可视化标定结果，评估标定质量
"""

import cv2
import numpy as np
import argparse
import sys
from pathlib import Path


class CalibrationAnalyzer:
    """标定结果分析器"""
    
    def __init__(self, yaml_file):
        """
        初始化分析器
        
        Args:
            yaml_file: YAML 标定文件路径
        """
        self.yaml_file = Path(yaml_file)
        self.result = {}
        self.load_calibration()
    
    def load_calibration(self):
        """从 YAML 文件加载标定结果"""
        if not self.yaml_file.exists():
            print(f"[错误] 文件不存在: {self.yaml_file}")
            sys.exit(1)
        
        try:
            fs = cv2.FileStorage(str(self.yaml_file), cv2.FILE_STORAGE_READ)
            self.result = {
                'image_width': int(fs.getNode('image_width').real()),
                'image_height': int(fs.getNode('image_height').real()),
                'image_count': int(fs.getNode('image_count').real()),
                'chessboard_cols': int(fs.getNode('chessboard_cols').real()),
                'chessboard_rows': int(fs.getNode('chessboard_rows').real()),
                'square_size_mm': fs.getNode('square_size_mm').real(),
                'reprojection_error': fs.getNode('reprojection_error').real(),
                'camera_matrix': fs.getNode('camera_matrix').mat(),
                'distortion_coefficients': fs.getNode('distortion_coefficients').mat(),
            }
            fs.release()
        except Exception as e:
            print(f"[错误] 无法加载文件: {e}")
            sys.exit(1)
    
    def print_summary(self):
        """打印标定结果摘要"""
        print("\n" + "="*50)
        print("相机标定结果摘要")
        print("="*50 + "\n")
        
        r = self.result
        
        print(f"图像分辨率: {r['image_width']} × {r['image_height']}")
        print(f"标定图像数: {r['image_count']}")
        print(f"棋盘格规格: {r['chessboard_cols']} × {r['chessboard_rows']}")
        print(f"方块边长: {r['square_size_mm']:.2f} mm")
        print(f"\n重投影误差: {r['reprojection_error']:.6f} px")
        self._print_error_quality(r['reprojection_error'])
        
        print("\n────── 相机矩阵 ──────")
        self._print_camera_matrix(r['camera_matrix'])
        
        print("\n────── 畸变系数 ──────")
        self._print_distortion(r['distortion_coefficients'])
    
    def _print_error_quality(self, error):
        """评估并打印误差质量"""
        if error < 0.5:
            quality = "优秀 ★★★★★"
        elif error < 1.0:
            quality = "良好 ★★★★"
        elif error < 2.0:
            quality = "可接受 ★★★"
        else:
            quality = "需要改进 ★★"
        
        print(f"评估: {quality}")
    
    def _print_camera_matrix(self, K):
        """打印相机矩阵详细信息"""
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        
        print(f"  fx (焦距X): {fx:.2f} px")
        print(f"  fy (焦距Y): {fy:.2f} px")
        print(f"  cx (光心X): {cx:.2f} px")
        print(f"  cy (光心Y): {cy:.2f} px")
        
        # 计算视角
        width = self.result['image_width']
        height = self.result['image_height']
        
        fov_x = 2 * np.arctan(width / (2 * fx)) * 180 / np.pi
        fov_y = 2 * np.arctan(height / (2 * fy)) * 180 / np.pi
        
        print(f"  视角X: {fov_x:.2f}°")
        print(f"  视角Y: {fov_y:.2f}°")
        print(f"\n  矩阵形式:\n{K}")
    
    def _print_distortion(self, dist):
        """打印畸变系数详细信息"""
        names = ['k1 (径向1)', 'k2 (径向2)', 'p1 (切向1)', 'p2 (切向2)', 'k3 (径向3)']
        
        for i, name in enumerate(names):
            if i < len(dist):
                val = dist[i, 0]
                print(f"  {name}: {val:.8f}")
    
    def check_validity(self):
        """检查标定结果的有效性"""
        print("\n" + "="*50)
        print("有效性检查")
        print("="*50 + "\n")
        
        checks = []
        
        # 检查1: 图像数量
        if self.result['image_count'] < 3:
            checks.append(("❌", "图像数量过少 (< 3)"))
        elif self.result['image_count'] < 10:
            checks.append(("⚠️ ", "建议增加标定图像 (建议 ≥ 20)"))
        else:
            checks.append(("✓", "图像数量充足"))
        
        # 检查2: 重投影误差
        error = self.result['reprojection_error']
        if error > 5.0:
            checks.append(("❌", f"重投影误差过大 ({error:.4f} px)"))
        elif error > 2.0:
            checks.append(("⚠️ ", f"重投影误差较大 ({error:.4f} px)"))
        elif error > 1.0:
            checks.append(("✓", f"重投影误差可接受 ({error:.4f} px)"))
        else:
            checks.append(("✓", f"重投影误差优秀 ({error:.4f} px)"))
        
        # 检查3: 焦距合理性
        fx = self.result['camera_matrix'][0, 0]
        fy = self.result['camera_matrix'][1, 1]
        width = self.result['image_width']
        
        if 0.3*width < fx < 5*width and 0.3*width < fy < 5*width:
            checks.append(("✓", f"焦距合理 (fx={fx:.0f}, fy={fy:.0f})"))
        else:
            checks.append(("⚠️ ", f"焦距值可能有问题 (fx={fx:.0f}, fy={fy:.0f})"))
        
        # 检查4: 光心位置
        cx = self.result['camera_matrix'][0, 2]
        cy = self.result['camera_matrix'][1, 2]
        
        if 0.1*width < cx < 0.9*width and 0.1*width < cy < 0.9*width:
            checks.append(("✓", f"光心位置合理 (cx={cx:.0f}, cy={cy:.0f})"))
        else:
            checks.append(("⚠️ ", f"光心位置可能有异常 (cx={cx:.0f}, cy={cy:.0f})"))
        
        # 检查5: 畸变系数
        dist = self.result['distortion_coefficients']
        if np.abs(dist).max() < 1.0:
            checks.append(("✓", "畸变系数在正常范围内"))
        else:
            checks.append(("⚠️ ", "畸变系数较大，相机可能有严重畸变"))
        
        # 打印检查结果
        for icon, message in checks:
            print(f"{icon} {message}")
        
        # 总体评估
        print("\n" + "-"*50)
        errors = sum(1 for icon, _ in checks if icon == "❌")
        warnings = sum(1 for icon, _ in checks if icon == "⚠️ ")
        
        if errors > 0:
            print("⚠️  存在严重问题，建议重新标定")
        elif warnings > 2:
            print("⚠️  存在多个警告，可能需要改进")
        else:
            print("✓ 标定结果良好，可以使用")
    
    def export_numpy(self, output_file):
        """导出为 NumPy 格式"""
        output_path = Path(output_file)
        
        data = {
            'camera_matrix': self.result['camera_matrix'],
            'distortion_coefficients': self.result['distortion_coefficients'],
        }
        
        np.save(output_path, data)
        print(f"\n[保存] NumPy 文件: {output_path}")
    
    def export_opencv(self, output_file):
        """导出为 OpenCV YAML 格式"""
        print(f"\n[保存] 已是 OpenCV 格式: {self.yaml_file}")
    
    def export_text(self, output_file):
        """导出为纯文本格式"""
        output_path = Path(output_file)
        
        with open(output_path, 'w') as f:
            f.write("="*50 + "\n")
            f.write("相机标定结果\n")
            f.write("="*50 + "\n\n")
            
            r = self.result
            f.write(f"分辨率: {r['image_width']}x{r['image_height']}\n")
            f.write(f"标定图像数: {r['image_count']}\n")
            f.write(f"棋盘格: {r['chessboard_cols']}x{r['chessboard_rows']}\n")
            f.write(f"方块大小: {r['square_size_mm']:.2f}mm\n")
            f.write(f"重投影误差: {r['reprojection_error']:.6f}px\n\n")
            
            f.write("相机矩阵:\n")
            f.write(str(r['camera_matrix']) + "\n\n")
            
            f.write("畸变系数:\n")
            f.write(str(r['distortion_coefficients']) + "\n")
        
        print(f"[保存] 文本文件: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="相机标定结果分析工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 查看标定结果
  python3 analyze_calibration.py camera_calibration.yaml
  
  # 导出为 NumPy 格式
  python3 analyze_calibration.py camera_calibration.yaml -n calibration.npy
        """)
    
    parser.add_argument('yaml_file', help='YAML 标定文件路径')
    parser.add_argument('-n', '--numpy', help='导出为 NumPy 文件')
    parser.add_argument('-t', '--text', help='导出为文本文件')
    parser.add_argument('-c', '--check-only', action='store_true',
                       help='仅进行有效性检查')
    
    args = parser.parse_args()
    
    analyzer = CalibrationAnalyzer(args.yaml_file)
    
    if not args.check_only:
        analyzer.print_summary()
    
    analyzer.check_validity()
    
    if args.numpy:
        analyzer.export_numpy(args.numpy)
    
    if args.text:
        analyzer.export_text(args.text)
    
    print("\n")


if __name__ == '__main__':
    main()
