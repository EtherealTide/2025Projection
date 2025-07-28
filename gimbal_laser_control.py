import numpy as np
import cv2
import time
from gimbal_control import *


class GimbalLaserControl:
    def __init__(self, gimbal: GIMBAL_CONTROL):
        """
        初始化云台激光控制类
        - scr_points: 屏幕坐标点
        - dst_points: 云台角度标定点
        - calibration_mode: 是否开启标定
        - calibration_step: 标定过程的状态
        """
        self.src_points = [
            [0, 0],
            [1.43, 0],
            [1.43, 1],
            [0, 1],
        ]  # 归一化屏幕坐标点，左下-右下-右上-左上
        # 云台角度标定点，初始化为四个点的坐标
        self.dst_points = [
            [4500, 4500],
            [5500, 4500],
            [5500, 5500],
            [4500, 5500],
        ]
        # 开启标定的标志位
        self.calibration_mode = True
        # 标定过程的状态变量
        self.calibration_step = "Finished"
        # 透视变换矩阵
        self.H = [[0] * 3 for _ in range(3)]  # 初始化为零矩阵
        self.gimbal = gimbal  # 云台控制实例

    def calibrate_gimbal(self):
        if self.calibration_mode:
            # 获取偏航角和俯仰角
            yaw, pitch = self.gimbal.yaw, self.gimbal.pitch
            if self.calibration_step == "First":
                self.dst_points[0] = [yaw, pitch]
                print("第一步标定完成")
            elif self.calibration_step == "Second":
                self.dst_points[1] = [yaw, pitch]
                print("第二步标定完成")
            elif self.calibration_step == "Third":
                self.dst_points[2] = [yaw, pitch]
                print("第三步标定完成")
            elif self.calibration_step == "Fourth":
                self.dst_points[3] = [yaw, pitch]
                print("第四步标定完成")
                # 计算透视变换矩阵
                self.H = cv2.getPerspectiveTransform(
                    np.array(self.src_points, dtype=np.float32),
                    np.array(self.dst_points, dtype=np.float32),
                )
                print("透视变换矩阵计算完成")
            self.calibration_mode = False

    # 执行透视变换
    def execute_perspective_transform(self, screen_points):
        """
        执行透视变换，将屏幕坐标 (x_s, y_s) 转换为云台角度 (theta, phi)
        """
        theta_list = []
        phi_list = []
        for i, (x_s, y_s) in enumerate(screen_points):
            theta, phi = perspective_transform(x_s, y_s, self.H)
            theta_list.append(theta)
            phi_list.append(phi)
        print(f"透视变换结果计算完成")
        return theta_list, phi_list

    # 云台执行动作
    def execute_gimbal_action(self, theta_list, phi_list, time_delay=0.04):

        for i in range(len(theta_list)):
            theta = theta_list[i]
            phi = phi_list[i]

            self.gimbal.set(theta, phi)

            self.gimbal.sendcmd()
            time_start = time.perf_counter_ns()
            my_delay_ms(time_delay * 1000)
            time_end = time.perf_counter_ns()
        print(f"云台动作执行完成")
        print(f"总延时: {(time_end - time_start) / 1e6:.2f} ms")

    # 计算图形周长并确定点数（单位长度50个点）
    def calculate_perimeter_and_points(self, shape_type, **kwargs):
        """
        计算图形周长并确定点数（单位长度50个点）

        参数:
        shape_type: 图形类型 ('triangle', 'rectangle', 'circle', 'star', 'sine')
        **kwargs: 各种图形的参数

        返回:
        perimeter: 周长
        points_count: 总点数
        """
        points_per_unit = 50  # 单位长度50个点

        if shape_type == "triangle":
            # 三角形: 计算三条边的长度
            x1, y1 = kwargs["x1"], kwargs["y1"]
            x2, y2 = kwargs["x2"], kwargs["y2"]
            x3, y3 = kwargs["x3"], kwargs["y3"]

            side1 = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            side2 = np.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)
            side3 = np.sqrt((x1 - x3) ** 2 + (y1 - y3) ** 2)

            perimeter = side1 + side2 + side3

        elif shape_type == "rectangle":
            # 矩形: 计算长和宽
            x1, y1 = kwargs["x1"], kwargs["y1"]
            x2, y2 = kwargs["x2"], kwargs["y2"]

            width = abs(x2 - x1)
            height = abs(y2 - y1)

            perimeter = 2 * (width + height)

        elif shape_type == "circle":
            # 圆形: 计算圆周长
            radius = kwargs["radius"]
            perimeter = 2 * np.pi * radius

        elif shape_type == "sine":
            # 正弦波: 使用积分近似计算弧长
            A = kwargs["A"]  # 振幅
            W = kwargs["W"]  # 波长

            # 正弦波弧长的近似计算
            # 使用数值积分方法
            x_points = np.linspace(0, W, 1000)
            dx = W / 1000

            arc_length = 0
            for i in range(len(x_points) - 1):
                dy_dx = A * 2 * np.pi / W * np.cos(2 * np.pi * x_points[i] / W)
                ds = np.sqrt(1 + dy_dx**2) * dx
                arc_length += ds

            perimeter = arc_length

        else:
            raise ValueError(f"不支持的图形类型: {shape_type}")

        # 计算总点数
        points_count = int(perimeter * points_per_unit)

        print(f"{shape_type} 周长: {perimeter:.4f}, 总点数: {points_count}")

        return perimeter, points_count

    # 修改现有的图形生成函数，使用计算出的点数
    def generate_triangle_points(self, x1, y1, x2, y2, x3, y3, steps=None):
        """
        生成三角形点序列
        """
        if steps is None:
            _, steps = self.calculate_perimeter_and_points(
                "triangle", x1=x1, y1=y1, x2=x2, y2=y2, x3=x3, y3=y3
            )

        # 计算每条边应该分配的点数
        side1_length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        side2_length = np.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)
        side3_length = np.sqrt((x1 - x3) ** 2 + (y1 - y3) ** 2)
        total_length = side1_length + side2_length + side3_length

        steps1 = int(steps * side1_length / total_length)
        steps2 = int(steps * side2_length / total_length)
        steps3 = steps - steps1 - steps2  # 剩余点数分配给第三边

        points = []
        # 第一条边
        for t in np.linspace(0, 1, steps1, endpoint=False):
            x = (1 - t) * x1 + t * x2
            y = (1 - t) * y1 + t * y2
            points.append((x, y))
        # 第二条边
        for t in np.linspace(0, 1, steps2, endpoint=False):
            x = (1 - t) * x2 + t * x3
            y = (1 - t) * y2 + t * y3
            points.append((x, y))
        # 第三条边
        for t in np.linspace(0, 1, steps3, endpoint=False):
            x = (1 - t) * x3 + t * x1
            y = (1 - t) * y3 + t * y1
            points.append((x, y))
        return points

    def generate_rectangle_points(self, x1, y1, x2, y2, steps=None):
        """
        生成矩形点序列
        """
        if steps is None:
            _, steps = self.calculate_perimeter_and_points(
                "rectangle", x1=x1, y1=y1, x2=x2, y2=y2
            )

        # 计算每条边的长度和对应点数
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        total_length = 2 * (width + height)

        steps_width = int(steps * width / total_length)
        steps_height = int(steps * height / total_length)

        points = []
        # 上边
        for t in np.linspace(0, 1, steps_width, endpoint=False):
            x = (1 - t) * x1 + t * x2
            y = y1
            points.append((x, y))
        # 右边
        for t in np.linspace(0, 1, steps_height, endpoint=False):
            x = x2
            y = (1 - t) * y1 + t * y2
            points.append((x, y))
        # 下边
        for t in np.linspace(0, 1, steps_width, endpoint=False):
            x = (1 - t) * x2 + t * x1
            y = y2
            points.append((x, y))
        # 左边
        for t in np.linspace(0, 1, steps_height, endpoint=False):
            x = x1
            y = (1 - t) * y2 + t * y1
            points.append((x, y))
        return points

    def generate_circle_points(self, center_x, center_y, radius, steps=None):
        """
        生成圆形点序列
        """
        if steps is None:
            _, steps = self.calculate_perimeter_and_points("circle", radius=radius)

        points = []
        for t in np.linspace(0, 2 * np.pi, steps, endpoint=False):
            x_s = center_x + radius * np.cos(t)
            y_s = center_y + radius * np.sin(t)
            points.append((x_s, y_s))
        return points

    def generate_sine_wave_points(self, A, y0, W, steps=None):
        """
        生成正弦波点序列
        """
        if steps is None:
            _, steps = self.calculate_perimeter_and_points("sine", A=A, W=W)

        points = []
        for x_s in np.linspace(0, W, steps):
            y_s = A * np.sin(2 * np.pi * x_s / W) + y0
            points.append((x_s, y_s))
        return points

    # 生成指定图形的点序列
    def read_point_from_file(self, file_path):
        """
        从文件中读取点序列，用于画画和写字，需要提前把指令文件放在文件系统
        """
        basic_delay_time = 2.2  # 基础延时时间，单位为ms
        with open(file_path, "r") as f:
            for line in f:
                if "ON" in line:
                    self.gimbal.ledstate = True  # 开启激光器
                    self.gimbal.sendcmd()
                elif "OFF" in line:
                    self.gimbal.ledstate = False  # 关闭激光器
                    self.gimbal.sendcmd()
                elif "M" in line:
                    x_s = float(line.split()[1])
                    y_s = float(line.split()[2])
                    theta, phi = perspective_transform(x_s, y_s, self.H)
                    dist_x = self.gimbal.yaw - theta
                    dist_y = self.gimbal.pitch - phi
                    self.gimbal.set(theta, phi)
                    self.gimbal.sendcmd()
                    my_delay_ms(
                        2.5 * max(abs(dist_x), abs(dist_y)) / basic_delay_time
                    )  # 延时，单位为毫秒


# 透视变换映射函数
def perspective_transform(x_s, y_s, H):
    """
    使用透视变换将屏幕坐标 (x_s, y_s) 映射到云台角度 (theta, phi)
    """
    point = np.array([[x_s, y_s, 1]], dtype=np.float32).T
    theta_phi_w = H @ point
    theta, phi, w = theta_phi_w.flatten()
    return theta / w, phi / w


def my_delay_ms(time_ms):
    """
    自定义延时函数，单位为毫秒
    """
    delay_mark = time.time()
    while True:
        if time.time() - delay_mark >= time_ms / 1000:
            break
