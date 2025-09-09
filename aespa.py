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
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.integral = 0  # Sum of errors over time
        self.previous_error = 0  # Previous error (used for derivative)

    def update(self, current_value):
        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        """ Update the target setpoint for the PID controller """
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike


class mecanum_drive:
    def __init__(self, deadzone=20):
        self.left_front = encoder_motor_class("M3", "INDEX1")  # Front Left
        self.left_back = encoder_motor_class("M6", "INDEX1")  # Front Right
        self.right_front = encoder_motor_class("M4", "INDEX1")  # Back Left
        self.right_back = encoder_motor_class("M2", "INDEX1")  # Back Right
        self.deadzone = deadzone
        self.vx_pid = PID(Kp=1, Ki=0, Kd=0, setpoint=0)
        self.vy_pid = PID(Kp=1, Ki=0, Kd=0.1, setpoint=0)

    def motor_drive(self, left_front_speed, left_back_speed, right_front_speed, right_back_speed):
        self.left_front.set_speed(left_front_speed * 0.95)
        self.left_back.set_speed(left_back_speed)
        self.right_front.set_speed(-right_front_speed * 0.9)
        self.right_back.set_speed(-right_back_speed * 0.9)

    def drive(self, velocity_x, velocity_y, angular_velocity):
        velocity_x *= 2.5
        velocity_y *= 2.5
        angular_velocity *= 2.5
        # Use deadzone to prevent motor from moving when joystick is not touched
        if math.fabs(velocity_x) < math.fabs(self.deadzone):
            velocity_x = 0
        if math.fabs(velocity_y) < math.fabs(self.deadzone):
            velocity_y = 0
        if math.fabs(angular_velocity) < math.fabs(self.deadzone):
            angular_velocity = 0
        # Calculation for the wheel speed
        # Visit https://github.com/neumann-lab/holonomic-mecanum/blob/main/th.md for the formula
        vFL = velocity_x + velocity_y - angular_velocity
        vFR = -(velocity_x) + velocity_y - angular_velocity
        vBL = -(velocity_x) + velocity_y + angular_velocity
        vBR = velocity_x + velocity_y + angular_velocity
        self.motor_drive(vFL, vFR, vBL, vBR)
    

class automatic:
    def __init__(self):
        pass


# ===== Manual Control Class =====
class manual:
    def __init__(self, mode, robot):
        self.top_convey = "DC1"
        self.under2_convey = "DC2"
        self.mid_convey = "DC3"
        self.under_convey = "DC6"
        self.gripper_port = encoder_motor_class("M5", "INDEX1")
        self.servo_level_adjustment = smartservo_class("M1", "INDEX1")
        self.mode = mode
        self.robot = robot
        self.BRL1 = False
        self.BRL2 = False
        self.plus_pressed = False
        self.menu_pressed = False
        self.normal_angle = -2.5
        self.max_current = 1.5
# pay 999999 robux for full codes
# see more full codes https://github.com/ximxal/Ultryn/blob/main/README.md
