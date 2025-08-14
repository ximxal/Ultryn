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


# ===== PID Class =====
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


# ===== Robot Control Class =====
class Robot:
    def __init__(self):
        # Motor mapping
        self.encoder_motor_M6 = encoder_motor_class("M3", "INDEX1")  # Front Left
        self.encoder_motor_M4 = encoder_motor_class("M6", "INDEX1")  # Front Right
        self.encoder_motor_M3 = encoder_motor_class("M4", "INDEX1")  # Back Left
        self.encoder_motor_M2 = encoder_motor_class("M2", "INDEX1")  # Back Right

        # Trim settings
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
        self.encoder_motor_M6.set_power(self.clamp_speed(fl * self.trim_M6))
        self.encoder_motor_M4.set_power(self.clamp_speed(fr * self.trim_M4))
        self.encoder_motor_M3.set_power(self.clamp_speed(bl * self.trim_M3))
        self.encoder_motor_M2.set_power(self.clamp_speed(br * self.trim_M2))

    def stop(self):
        self.set_motor_speed(0, 0, 0, 0)

    def control_system(self):
        lx = self.apply_deadzone(gamepad.get_joystick("Lx"))
        ly = self.apply_deadzone(gamepad.get_joystick("Ly"))
        rx = self.apply_deadzone(gamepad.get_joystick("Rx"))

        vx, vy, vw = lx, ly, -rx

        if self.enable_pid:
            vx = self.pid_x.update(vx)
            vy = self.pid_y.update(vy)
            vw = self.pid_r.update(vw)

        if gamepad.is_key_pressed("Left"):
            self.set_motor_speed(-50, 50, 50, -50)
        elif gamepad.is_key_pressed("Right"):
            self.set_motor_speed(50, -50, -50, 50)
        elif vx != 0 or vy != 0 or vw != 0:
            fl = vy + vw - vx
            fr = vy - vw + vx
            bl = vy - vw - vx
            br = vy + vw + vx
            self.set_motor_speed(fl, fr, bl, br)
        else:
            self.stop()


# ===== Manual Control Class =====
class manual:
    def __init__(self, mode):
        self.top_convey = "DC1"
        self.under2_convey = "DC2"
        self.mid_convey = "DC3"
        self.under_convey = "DC6"
        self.gripper_port = encoder_motor_class("M5", "INDEX1")
        self.servo_level_adjustment = smartservo_class("M1", "INDEX1")
        self.mode = mode
        self.BRL1 = False
        self.BRL2 = False
        self.plus_pressed = False
        self.menu_pressed = False
        self.normal_angle = -2.5
        self.max_current = 1.5

    def shooter(self):
        dc_speed = 100

        current_val = self.servo_level_adjustment.get_value("current")
        if current_val >= self.max_current:
            self.servo_level_adjustment.move_to(self.normal_angle, 50)

        if gamepad.is_key_pressed("N2"):
            power_expand_board.set_power(self.under_convey, -dc_speed)
            power_expand_board.set_power(self.under2_convey, -dc_speed)
            power_expand_board.set_power(self.mid_convey, -dc_speed)
            power_expand_board.set_power(self.top_convey, 0)
        elif gamepad.is_key_pressed("R1"):
            power_expand_board.set_power(self.under_convey, -dc_speed)
            power_expand_board.set_power(self.under2_convey, -dc_speed)
            power_expand_board.set_power(self.mid_convey, -dc_speed)
            power_expand_board.set_power(self.top_convey, dc_speed)
        elif gamepad.is_key_pressed("L1"):
            power_expand_board.set_power(self.under_convey, dc_speed)
            power_expand_board.set_power(self.under2_convey, dc_speed)
            power_expand_board.set_power(self.mid_convey, dc_speed)
            power_expand_board.set_power(self.top_convey, -dc_speed)
        else:
            power_expand_board.set_power(self.under_convey, 0)
            power_expand_board.set_power(self.under2_convey, 0)
            power_expand_board.set_power(self.mid_convey, 0)
            power_expand_board.set_power(self.top_convey, 0)

        # Servo adjustments
        if gamepad.is_key_pressed("Up"):
            self.servo_level_adjustment.move_to(-15, 80)
        if gamepad.is_key_pressed("Down"):
            self.servo_level_adjustment.move_to(3, 80)
        if gamepad.is_key_pressed("R_Thumb"):
            self.servo_level_adjustment.move(5, 25)
        if gamepad.is_key_pressed("L_Thumb"):
            self.servo_level_adjustment.move_to(self.normal_angle, 25)


        if gamepad.is_key_pressed("+") and not self.plus_pressed:
            self.BRL1 = not self.BRL1
            self.BRL2 = False
            power_expand_board.set_power("BL1", 100 if self.BRL1 else 0)
            power_expand_board.set_power("BL2", 0)
            self.plus_pressed = True
        elif not gamepad.is_key_pressed("+"):
            self.plus_pressed = False

        if gamepad.is_key_pressed("≡") and not self.menu_pressed:
            self.BRL2 = not self.BRL2
            self.BRL1 = False
            if self.BRL2:
                power_expand_board.set_power("BL1", 100)
                power_expand_board.set_power("BL2", 100)
            else:
                power_expand_board.set_power("BL1", 0)
                power_expand_board.set_power("BL2", 0)
            self.menu_pressed = True
        elif not gamepad.is_key_pressed("≡"):
            self.menu_pressed = False


robot = Robot()
manual_mode = manual("manual")

while True:
    time.sleep(0.01)
    if not power_manage_module.is_auto_mode():
        robot.control_system()
        manual_mode.shooter()
