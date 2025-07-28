from maix import camera, display, image, nn, app
from state import STATE
from screen_state import SCREEN
import threading, time
from gimbal_control import GIMBAL_CONTROL
from gimbal_laser_control import GimbalLaserControl
from parameter_control import ParameterController

CAMSIZE = [640, 480]

# 初始化
state = STATE()
cam = camera.Camera(
    CAMSIZE[0],
    CAMSIZE[1],
)
disp = display.Display()
# 初始化云台控制
gimbal = GIMBAL_CONTROL(state, port="/dev/ttyS0", baudrate=115200, maxlen=8)
# 初始化激光绘制
gimbal_laser = GimbalLaserControl(gimbal)
screen = SCREEN(state, gimbal_laser, gimbal)
# 启动屏幕监听线程
thread_screen = threading.Thread(target=screen.listen)
thread_screen.daemon = True
thread_screen.start()

# 初始化参数控制器
param_controller = ParameterController(gimbal_laser=gimbal_laser)
# 加载参数
param_controller.load_parameters()

# 主循环
while state.running and not app.need_exit():
    img = cam.read()
    if state.workmode == "init":
        img_show = screen.draw_button(img)
    elif state.workmode == "cali":
        img_show = screen.draw_button()
    elif state.workmode == "setp":
        img_show = screen.draw_button()
    # 将yaw和pitch显示在屏幕上
    img_show.draw_string(
        340,
        10,
        f"Yaw: {gimbal.yaw}, Pitch: {gimbal.pitch}",
        color=image.Color.from_rgb(0, 0, 255),
        scale=2,
        thickness=1,
    )
    disp.show(img_show)
# 退出时保存参数
param_controller.save_parameters()
