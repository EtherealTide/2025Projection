import os
from gimbal_laser_control import GimbalLaserControl
import cv2
import numpy as np


class ParameterController:
    """参数控制器类，管理摄像头参数调节"""

    def __init__(
        self,
        param_file_path="/root/user/2025projection.txt",
        gimbal_laser: GimbalLaserControl = None,
    ):
        self.param_file_path = param_file_path
        self.gimbal_laser = gimbal_laser

    def load_parameters(self):
        """从文件加载参数"""
        try:
            if os.path.exists(self.param_file_path):
                with open(self.param_file_path, "r") as file:
                    content = file.read().strip()
                    if content:
                        parts = content.split(",")
                        for i, part in enumerate(parts):
                            self.gimbal_laser.dst_points[i // 2][i % 2] = part
                        print(
                            f"[PARAM] 标定点坐标参数已加载: {self.gimbal_laser.dst_points}"
                        )
                        self.gimbal_laser.H = cv2.getPerspectiveTransform(
                            np.array(self.gimbal_laser.src_points, dtype=np.float32),
                            np.array(self.gimbal_laser.dst_points, dtype=np.float32),
                        )
                        print("[PARAM] 透视变换矩阵已计算")
                        return True

                    else:
                        print("[PARAM] 文件为空，使用默认值")
            else:
                print("[PARAM] 参数文件不存在，使用默认值")

        except Exception as e:
            print(f"[PARAM] 读取参数文件失败: {e}，使用默认值")

        # 如果读取失败，使用默认值
        self.gimbal_laser.dst_points = [
            [4500, 4500],
            [5500, 4500],
            [5500, 5500],
            [4500, 5500],
        ]
        return False

    def save_parameters(self):
        """保存参数到文件"""
        try:
            # 确保目录存在
            os.makedirs(os.path.dirname(self.param_file_path), exist_ok=True)

            # 写入分辨率参数
            with open(self.param_file_path, "w") as file:
                content = ",".join(
                    [
                        str(coord)
                        for point in self.gimbal_laser.dst_points
                        for coord in point
                    ]
                )
                file.write(content)
                print(f"[PARAM] 标定点坐标参数已保存: {self.gimbal_laser.dst_points}")
            return True

        except Exception as e:
            print(f"[PARAM] 保存参数文件失败: {e}")
            return False
