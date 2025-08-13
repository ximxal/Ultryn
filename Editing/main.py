import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.led_matrix import led_matrix_class
from mbuild.smart_camera import smart_camera_class
from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0


class Robot:
    def __init__(self):
        # กำหนดมอเตอร์ตามพอร์ต
        self.encoder_motor_M6 = encoder_motor_class("M3", "INDEX1")  # Front Left
        self.encoder_motor_M4 = encoder_motor_class("M6", "INDEX1")  # Front Right
        self.encoder_motor_M3 = encoder_motor_class("M4", "INDEX1")  # Back Left
        self.encoder_motor_M2 = encoder_motor_class("M2", "INDEX1")  # Back Right

        # trim ปรับสมดุลล้อ
        self.trim_M6 = 0.90   # FL
        self.trim_M4 = -1.10  # FR
        self.trim_M3 = -0.90  # BL
        self.trim_M2 = 0.90   # BR

        self.max_speed = 100
        self.deadzone = 15

        # PID
        self.enable_pid = False
        self.pid_x = PID(Kp=1.0, Ki=0.0, Kd=0.1)
        self.pid_y = PID(Kp=1.0, Ki=0.0, Kd=0.1)
        self.pid_r = PID(Kp=1.0, Ki=0.0, Kd=0.1)

    def clamp_speed(self, value):
        return max(min(int(value), self.max_speed), -self.max_speed)

    def apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0

    def set_motor_speed(self, fl, fr, bl, br):
        self.encoder_motor_M6.set_power(self.clamp_speed(fl * self.trim_M6))  # FL
        self.encoder_motor_M4.set_power(self.clamp_speed(fr * self.trim_M4))  # FR
        self.encoder_motor_M3.set_power(self.clamp_speed(bl * self.trim_M3))  # BL
        self.encoder_motor_M2.set_power(self.clamp_speed(br * self.trim_M2))  # BR

    def stop(self):
        self.set_motor_speed(0, 0, 0, 0)

    def control_system(self):
        # อ่านค่าจอยสติ๊ก
        lx = self.apply_deadzone(gamepad.get_joystick("Lx"))
        ly = self.apply_deadzone(gamepad.get_joystick("Ly"))
        rx = self.apply_deadzone(gamepad.get_joystick("Rx"))

        vx = lx
        vy = ly
        vw = -rx

        if self.enable_pid:
            vx = self.pid_x.update(vx)
            vy = self.pid_y.update(vy)
            vw = self.pid_r.update(vw)

        if gamepad.is_key_pressed("Left"):
            self.set_motor_speed(-50, 50, 50, -50)
        elif gamepad.is_key_pressed("Right"):
            self.set_motor_speed(50, -50, -50, 50)
        elif vx != 0 or vy != 0 or vw != 0:
            # คำนวณความเร็วล้อ (NovaPi Mecanum)
            fl = vy + vw - vx   # Front Left
            fr = vy - vw + vx   # Front Right
            bl = vy - vw - vx   # Back Left
            br = vy + vw + vx   # Back Right
            self.set_motor_speed(fl, fr, bl, br)
        else:
            self.stop()


class dc_motor:
    def __init__(self, motor_port):
        self.motor_port = motor_port

    def set_power(self, power):
        power_expand_board.set_power(self.motor_port, power)

    def enable(self, inverse=False):
        power_expand_board.set_power(self.motor_port, -100 if inverse else 100)

    def disable(self):
        power_expand_board.set_power(self.motor_port, 0)


top_feed = dc_motor("DC1")
under2_feed = dc_motor("DC2")
mid_feed = dc_motor("DC3")
under_feed = dc_motor("DC6")
Gripper_port = encoder_motor_class("M5", "INDEX1")
servo_level_adjustment = smartservo_class("M1", "INDEX1")


class manual:
    def __init__(self, mode):
        self.mode = mode
        self.brushless = False

    def mission(self):
        if gamepad.is_key_pressed("N2"):
            under_feed.set_power(-100)
            under2_feed.set_power(-100)
            mid_feed.set_power(-100)
        if gamepad.is_key_pressed("R1"):
            under_feed.set_power(-100)
            under2_feed.set_power(-100)
            mid_feed.set_power(-100)
            top_feed.set_power(100)
        elif gamepad.is_key_pressed("L1"):
            under_feed.set_power(100)
            under2_feed.set_power(100)
            mid_feed.set_power(100)
            top_feed.set_power(-100)
        if gamepad.is_key_pressed("Up"):
            servo_level_adjustment.move_to(-15, 80)
        if gamepad.is_key_pressed("Down"):
            servo_level_adjustment.move_to(3, 80)
        if gamepad.is_key_pressed("R_Thumb"):
            servo_level_adjustment.move(5, 25)
        if gamepad.is_key_pressed("L_Thumb"):
            servo_level_adjustment.move_to(-2.5, 25)
            

robot = Robot()

while True:
    time.sleep(0.01)
    if not power_manage_module.is_auto_mode():
        robot.control_system()
