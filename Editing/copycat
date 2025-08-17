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


class robot:
    def __init__(self, movement: mecanum_drive, manual_stage: manual, automatic_stage: automatic):
        self.movement = movement
        self.manual = manual_stage
        self.automatic = automatic_stage
    
    def joystick_control(self):
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            self.movement.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), gamepad.get_joystick("Rx"))
        else:
            self.movement.motor_drive(0, 0, 0, 0)
        # if math.fabs(gamepad.get_joystick("Ry")) > 50:
            # lift.set_speed(gamepad.get_joystick("Ry"))
        # else:
           # lift.set_speed(0)

    def handle_manual_stage(self):
        self.joystick_control()
        self.manual.shooter()
    
    def handle_automatic_stage(self):
        time.sleep(0.5)

robot = robot(mecanum_drive(), manual(), automatic())

while True:
    if power_manage_module.is_auto_mode():
        robot.handle_automatic_stage()
        while power_manage_module.is_auto_mode():
            pass
    else:
        robot.handle_manual_stage()
