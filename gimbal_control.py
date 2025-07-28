from maix import uart, pinmap
import struct, time, threading
from collections import deque
from state import *

# 舵机运动参数配置
PGAP = 500  # 俯仰角（pitch）步进值
YGAP = 20  # 偏航角（yaw）步进值
YAWLIM = [3000, 7000]  # 偏航角范围
PITCHLIM = [3000, 7000]  # 俯仰角范围


class GIMBAL_CONTROL:
    """
    串口控制类 - 用于控制二轴云台的俯仰和偏航运动

    主要功能：
    - 控制云台的yaw（偏航）和pitch（俯仰）运动
    - 控制激光器开关和亮度
    - 控制蜂鸣器
    - 自动扫描模式
    - 射击检测和控制
    """

    def __init__(
        self,
        state: STATE,
        port: str = "/dev/ttyS1",
        baudrate: int = 115200,
        maxlen: int = 8,
    ) -> None:
        """
        初始化串口控制器

        输入参数:
            state: STATE对象，用于状态管理
            port: 串口设备路径，默认"/dev/ttyS1"
            baudrate: 波特率，默认115200
            maxlen: 历史数据队列最大长度，默认8

        输出: 无

        调用场景: 程序启动时初始化云台控制
        """
        # 配置串口引脚
        # pinmap.set_pin_function("A18", "UART1_RX")
        # pinmap.set_pin_function("A19", "UART1_TX")

        # 初始化成员变量
        self.STATE = state
        self.ledstate = False  # 激光器状态
        self.ser = uart.UART(port, baudrate)

        # 云台角度控制参数（范围3000-7000，中心5000）
        self.yaw: int = 5000  # 当前偏航角
        self.pitch: int = 5000  # 当前俯仰角
        self.oldyaw: int = 5000  # 上一次偏航角
        self.oldpitch: int = 5000  # 上一次俯仰角

        # 历史数据队列，用于平滑控制
        self.yaw_li = deque(maxlen=maxlen)
        self.pitch_li = deque(maxlen=maxlen)

        # 初始化历史数据队列
        for i in range(7):
            self.yaw_li.append(5000)
            self.pitch_li.append(5000)

        # 系统启动提示音
        for i in range(12):
            if i % 2 == 0 and i > 9:
                self.STATE.speaker = True
            else:
                self.STATE.speaker = False
            self.sendcmd()
            time.sleep(0.15)
        self.STATE.speaker = False

        print("Serial is open!")

    def close(self):
        """
        关闭串口连接

        输入参数: 无
        输出: 无

        调用场景: 程序退出时清理资源
        """
        print("Closing serial!")
        # 复位到中心位置
        self.yaw = 5000
        self.pitch = 5500
        self.ledstate = False
        self.sendcmd()
        self.ser.close()

    def set(self, yaw, pitch):
        """
        设置云台目标角度

        输入参数:
            yaw: 目标偏航角（3000-7000）
            pitch: 目标俯仰角（3000-7000）

        输出: 无

        调用场景: 根据视觉检测结果调整云台指向
        """
        # 限制角度范围，防止超出机械极限
        self.yaw = max(YAWLIM[0], min(YAWLIM[1], int(yaw)))
        self.pitch = max(PITCHLIM[0], min(PITCHLIM[1], int(pitch)))

    def sendcmd(self):
        """
        发送控制命令到下位机

        输入参数: 无
        输出: 无

        调用场景: 每次需要更新云台状态时调用

        协议格式: 'AA' + yaw(2字节) + pitch(2字节) + led(1字节) + light(1字节) + speaker(1字节) + checksum(1字节)
        """
        # 创建数据帧 - yaw需要反向映射
        yaw_bytes = struct.pack("<H", -(self.yaw - 5000) + 5000)
        pitch_bytes = struct.pack("<H", self.pitch)

        # 激光器控制
        if self.ledstate:
            led_byte = 0x50.to_bytes(1, "big")  # 开启
        else:
            led_byte = 0x00.to_bytes(1, "big")

        # 蜂鸣器控制
        if self.STATE.speaker:
            speaker_byte = 0x01.to_bytes(1, "big")  # 开启
        else:
            speaker_byte = 0x00.to_bytes(1, "big")
        # 组装数据帧
        frame = 0xAA.to_bytes(1, "big") + 0x06.to_bytes(1, "big")
        frame += yaw_bytes + pitch_bytes + led_byte + speaker_byte

        # 计算校验和
        checksum = 0
        checksum = sum(frame[1:8]) & 0xFF
        frame += struct.pack("B", checksum) + 0xBB.to_bytes(1, "big")
        # 发送数据帧
        self.ser.write(frame)
        # 更新历史数据
        self.yaw_li.append(self.yaw)
        self.pitch_li.append(self.pitch)

    def gomid(self):
        """
        云台回到中心位置

        输入参数: 无
        输出: 无

        调用场景: 复位云台到初始位置
        """
        self.yaw = 5000  # 中心偏航角
        self.pitch = 5000  # 中心俯仰角

        # 执行5次命令确保到位，最后一次有提示音
        for i in range(5):
            if i == 4:
                self.STATE.speaker = True
            else:
                self.STATE.speaker = False
            self.sendcmd()
            time.sleep(0.15)
        self.STATE.speaker = False
