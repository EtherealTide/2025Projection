from maix import touchscreen, app, image
from state import *
import time
from gimbal_laser_control import GimbalLaserControl
from gimbal_control import GIMBAL_CONTROL
import numpy as np

SCR_SIZE = (640, 480)
BUT_SIZE = (80, 60)


class SCREEN:
    button_init = {
        "exit": [10, 10, BUT_SIZE[0], BUT_SIZE[1]],
        "cali": [10, 100, BUT_SIZE[0], BUT_SIZE[1]],
        "sine": [10, 200, BUT_SIZE[0], BUT_SIZE[1]],
        "triangle": [10, 300, BUT_SIZE[0], BUT_SIZE[1]],
        "rectangle": [100, 200, BUT_SIZE[0], BUT_SIZE[1]],
        "circle": [100, 300, BUT_SIZE[0], BUT_SIZE[1]],
        # 五角星
        "file": [200, 200, BUT_SIZE[0], BUT_SIZE[1]],
    }
    button_cali = {
        "back": [10, 10, BUT_SIZE[0], BUT_SIZE[1]],
        "set1": [50, 150, BUT_SIZE[0], BUT_SIZE[1]],
        "set2": [200, 150, BUT_SIZE[0], BUT_SIZE[1]],
        "set3": [350, 150, BUT_SIZE[0], BUT_SIZE[1]],
        "set4": [500, 150, BUT_SIZE[0], BUT_SIZE[1]],
        "go1": [50, 300, BUT_SIZE[0], BUT_SIZE[1]],
        "go2": [200, 300, BUT_SIZE[0], BUT_SIZE[1]],
        "go3": [350, 300, BUT_SIZE[0], BUT_SIZE[1]],
        "go4": [500, 300, BUT_SIZE[0], BUT_SIZE[1]],
    }
    button_setp = {
        "back": [10, 10, BUT_SIZE[0], BUT_SIZE[1]],
        "x-100": [10, 200, BUT_SIZE[0], BUT_SIZE[1]],
        "x-10": [110, 200, BUT_SIZE[0], BUT_SIZE[1]],
        "x-1": [210, 200, BUT_SIZE[0], BUT_SIZE[1]],
        "x+1": [310, 200, BUT_SIZE[0], BUT_SIZE[1]],
        "x+10": [410, 200, BUT_SIZE[0], BUT_SIZE[1]],
        "x+100": [510, 200, BUT_SIZE[0], BUT_SIZE[1]],
        "y+100": [10, 100, BUT_SIZE[0], BUT_SIZE[1]],
        "y+10": [210, 100, BUT_SIZE[0], BUT_SIZE[1]],
        "y+1": [410, 100, BUT_SIZE[0], BUT_SIZE[1]],
        "y-1": [10, 300, BUT_SIZE[0], BUT_SIZE[1]],
        "y-10": [210, 300, BUT_SIZE[0], BUT_SIZE[1]],
        "y-100": [410, 300, BUT_SIZE[0], BUT_SIZE[1]],
        "save": [550, 400, BUT_SIZE[0], BUT_SIZE[1]],
    }

    def inbutton(self, x, y, button):
        if (
            x > button[0]
            and x < button[0] + button[2]
            and y > button[1]
            and y < button[1] + button[3]
        ):
            return True
        else:
            return False

    def __init__(
        self, state: STATE, gimbal_laser: GimbalLaserControl, gimbal: GIMBAL_CONTROL
    ) -> None:
        self.ts = touchscreen.TouchScreen()
        self.state = state
        self.last_x = 0
        self.last_y = 0
        self.img_null = image.Image(SCR_SIZE[0], SCR_SIZE[1])
        self.gimbal_laser = gimbal_laser
        self.gimbal = gimbal
        self.img_null.draw_rect(
            0, 0, SCR_SIZE[0], SCR_SIZE[1], image.COLOR_WHITE, thickness=-1
        )

    def gimbal_laser_init(self, theta_first, phi_first):
        self.state.speaker = True
        # 发送第一个点
        self.gimbal.set(theta_first, phi_first)
        self.gimbal.sendcmd()
        time.sleep(0.1)
        self.state.speaker = False
        self.gimbal.sendcmd()
        time.sleep(1)
        self.gimbal.ledstate = True  # 开启激光器
        self.gimbal.sendcmd()

    def listen(self):
        while self.state.running:
            x, y, pressed = self.ts.read()
            if (x != self.last_x or y != self.last_y) and pressed == 1:
                self.last_x = x
                self.last_y = y
                self.pressprocess(x, y)

            time.sleep(0.01)

    def pressprocess(self, x, y):
        if self.state.workmode == "init":
            if self.inbutton(x, y, self.button_init["exit"]):
                print("exit")
                self.state.running = False
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_init["cali"]):
                print("cali")
                self.state.workmode = "cali"
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_init["sine"]):
                if not np.array_equal(self.gimbal_laser.H, np.zeros((3, 3))):
                    print("sine")
                    points = self.gimbal_laser.generate_sine_wave_points(0.5, 0.5, 1)
                    theta_list, phi_list = (
                        self.gimbal_laser.execute_perspective_transform(points)
                    )
                    self.gimbal_laser_init(theta_list[0], phi_list[0])
                    self.gimbal_laser.execute_gimbal_action(theta_list, phi_list)
                    # 关闭激光器
                    self.gimbal.ledstate = False
                    self.gimbal.sendcmd()
                    time.sleep(0.05)
                else:
                    print("calibration not done")
            elif self.inbutton(x, y, self.button_init["triangle"]):
                if not np.array_equal(self.gimbal_laser.H, np.zeros((3, 3))):
                    print("triangle")
                    points = self.gimbal_laser.generate_triangle_points(
                        0.2, 0.2, 0.8, 0.2, 0.5, 0.8
                    )
                    theta_list, phi_list = (
                        self.gimbal_laser.execute_perspective_transform(points)
                    )
                    self.gimbal_laser_init(theta_list[0], phi_list[0])
                    self.gimbal_laser.execute_gimbal_action(theta_list, phi_list)
                    # 关闭激光器
                    self.gimbal.ledstate = False
                    self.gimbal.sendcmd()
                    time.sleep(0.05)
                else:
                    print("calibration not done")
            elif self.inbutton(x, y, self.button_init["rectangle"]):
                if not np.array_equal(self.gimbal_laser.H, np.zeros((3, 3))):
                    print("rectangle")
                    points = self.gimbal_laser.generate_rectangle_points(
                        0.2, 0.2, 0.8, 0.8
                    )
                    theta_list, phi_list = (
                        self.gimbal_laser.execute_perspective_transform(points)
                    )
                    self.gimbal_laser_init(theta_list[0], phi_list[0])
                    self.gimbal_laser.execute_gimbal_action(theta_list, phi_list)
                    # 关闭激光器
                    self.gimbal.ledstate = False
                    self.gimbal.sendcmd()
                    time.sleep(0.05)
                else:
                    print("calibration not done")
            elif self.inbutton(x, y, self.button_init["circle"]):
                if not np.array_equal(self.gimbal_laser.H, np.zeros((3, 3))):
                    print("circle")
                    points = self.gimbal_laser.generate_circle_points(0.5, 0.5, 0.4)
                    theta_list, phi_list = (
                        self.gimbal_laser.execute_perspective_transform(points)
                    )
                    self.gimbal_laser_init(theta_list[0], phi_list[0])
                    self.gimbal_laser.execute_gimbal_action(theta_list, phi_list)
                    # 关闭激光器
                    self.gimbal.ledstate = False
                    self.gimbal.sendcmd()
                    time.sleep(0.05)
                else:
                    print("calibration not done")
            elif self.inbutton(x, y, self.button_init["file"]):
                if not np.array_equal(self.gimbal_laser.H, np.zeros((3, 3))):
                    print("file")
                    # while True:
                    self.gimbal_laser.read_point_from_file("/root/user/et41.txt")
                    time.sleep(0.05)
                else:
                    print("calibration not done")
        elif self.state.workmode == "cali":
            self.gimbal.ledstate = True  # 开启激光器
            self.gimbal.sendcmd()
            if self.inbutton(x, y, self.button_cali["back"]):
                print("back")
                self.state.workmode = "init"
                self.gimbal_laser.calibration_step = (
                    "Finished"  # 退回主界面时说明校正完成
                )
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_cali["set1"]):
                print("set1")
                self.gimbal_laser.calibration_step = "First"
                self.state.workmode = "setp"
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_cali["set2"]):
                print("set2")
                self.gimbal_laser.calibration_step = "Second"
                self.state.workmode = "setp"
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_cali["set3"]):
                print("set3")
                self.gimbal_laser.calibration_step = "Third"
                self.state.workmode = "setp"
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_cali["set4"]):
                print("set4")
                self.gimbal_laser.calibration_step = "Fourth"
                self.state.workmode = "setp"
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_cali["go1"]):
                if self.gimbal_laser.dst_points[0] != [0, 0]:
                    print("go1")
                    self.gimbal.set(
                        self.gimbal_laser.dst_points[0][0],
                        self.gimbal_laser.dst_points[0][1],
                    )
                    self.gimbal.sendcmd()
                else:
                    print("go1 not set")
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_cali["go2"]):
                if self.gimbal_laser.dst_points[1] != [0, 0]:
                    print("go2")
                    self.gimbal.set(
                        self.gimbal_laser.dst_points[1][0],
                        self.gimbal_laser.dst_points[1][1],
                    )
                    self.gimbal.sendcmd()
                else:
                    print("go2 not set")
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_cali["go3"]):
                if self.gimbal_laser.dst_points[2] != [0, 0]:
                    print("go3")
                    self.gimbal.set(
                        self.gimbal_laser.dst_points[2][0],
                        self.gimbal_laser.dst_points[2][1],
                    )
                    self.gimbal.sendcmd()
                else:
                    print("go3 not set")
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_cali["go4"]):
                if self.gimbal_laser.dst_points[3] != [0, 0]:
                    print("go4")
                    self.gimbal.set(
                        self.gimbal_laser.dst_points[3][0],
                        self.gimbal_laser.dst_points[3][1],
                    )
                    self.gimbal.sendcmd()
                else:
                    print("go4 not set")
                time.sleep(0.05)
        elif self.state.workmode == "setp":
            if self.inbutton(x, y, self.button_setp["back"]):
                print("back")
                self.state.workmode = "cali"
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["save"]):
                self.gimbal_laser.calibration_mode = True  # 开启标定
                print("save")
                self.state.workmode = "cali"
                self.gimbal_laser.calibrate_gimbal()  # 执行标定
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["x-100"]):
                print("x-100")
                self.gimbal.set(self.gimbal.yaw - 100, self.gimbal.pitch)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["x-10"]):
                print("x-10")
                self.gimbal.set(self.gimbal.yaw - 10, self.gimbal.pitch)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["x-1"]):
                print("x-1")
                self.gimbal.set(self.gimbal.yaw - 1, self.gimbal.pitch)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["x+1"]):
                print("x+1")
                self.gimbal.set(self.gimbal.yaw + 1, self.gimbal.pitch)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["x+10"]):
                print("x+10")
                self.gimbal.set(self.gimbal.yaw + 10, self.gimbal.pitch)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["x+100"]):
                print("x+100")
                self.gimbal.set(self.gimbal.yaw + 100, self.gimbal.pitch)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["y+100"]):
                print("y+100")
                self.gimbal.set(self.gimbal.yaw, self.gimbal.pitch + 100)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["y+10"]):
                print("y+10")
                self.gimbal.set(self.gimbal.yaw, self.gimbal.pitch + 10)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["y+1"]):
                print("y+1")
                self.gimbal.set(self.gimbal.yaw, self.gimbal.pitch + 1)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["y-1"]):
                print("y-1")
                self.gimbal.set(self.gimbal.yaw, self.gimbal.pitch - 1)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["y-10"]):
                print("y-10")
                self.gimbal.set(self.gimbal.yaw, self.gimbal.pitch - 10)
                self.gimbal.sendcmd()
                time.sleep(0.05)
            elif self.inbutton(x, y, self.button_setp["y-100"]):
                print("y-100")
                self.gimbal.set(self.gimbal.yaw, self.gimbal.pitch - 100)
                self.gimbal.sendcmd()
                time.sleep(0.05)

    def draw_button(self, img=None):
        if img is None:
            img = image.Image(SCR_SIZE[0], SCR_SIZE[1])
            # img.draw_rect(0, 0, SCR_SIZE[0], SCR_SIZE[1], image.COLOR_WHITE,thickness=-1)
        zoom_kx = img.width() / SCR_SIZE[0]
        zoom_ky = img.height() / SCR_SIZE[1]
        if self.state.workmode == "init":
            butlist = self.button_init
        elif self.state.workmode == "cali":
            butlist = self.button_cali
        elif self.state.workmode == "setp":
            butlist = self.button_setp

        for butstr in butlist:
            but = butlist[butstr]
            img.draw_rect(
                round(but[0] * zoom_kx),
                round(but[1] * zoom_ky),
                round(but[2] * zoom_kx),
                round(but[3] * zoom_ky),
                image.COLOR_RED,
                thickness=-1,
            )
            img.draw_string(
                round(but[0] * zoom_kx),
                round(but[1] * zoom_ky) + 10,
                butstr,
                image.COLOR_WHITE,
                scale=1.5,
                thickness=2,
            )
        return img

    def inbutton(self, x, y, button):
        return (
            button[0] < x <= button[0] + button[2]
            and button[1] < y <= button[1] + button[3]
        )
