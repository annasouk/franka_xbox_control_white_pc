from inputs import get_gamepad
import math
import threading
import time
from franky import *
import numpy as np
from scipy.spatial.transform import Rotation

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
        self.robot = Robot("192.168.0.2")
        self.robot.recover_from_errors()
        self.robot.relative_dynamics_factor = 0.05


    def read(self): # return the buttons/triggers that you care about in this methode
        left_x = self.LeftJoystickX
        left_y = self.LeftJoystickY
        right_z = self.RightJoystickY

        roll_neg = self.X
        roll_pos = self.B
        roll = roll_pos - roll_neg


        pitch_neg = self.Y
        pitch_pos = self.A
        pitch = pitch_pos - pitch_neg

        yaw_neg = self.LeftBumper
        yaw_pos = self.RightBumper
        yaw = yaw_pos - yaw_neg
        left_trigger = self.LeftTrigger
        right_trigger = self.RightTrigger

        # print(f"{roll_neg=}")
        # print(f"{roll_pos=}")
        # print(f"{pitch_pos=}")
        # print(f"{pitch_neg=}")
        # print(f"{yaw_pos=}")
        # print(f"{yaw_neg=}")


        return [left_x, left_y, right_z, roll, pitch, yaw, left_trigger, right_trigger]


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_EAST':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_SOUTH':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state




if __name__ == '__main__':
    joy = XboxController()
    home = JointMotion([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    joy.robot.move(home)

    while True:
        multiplier = 10.0 / 100.0
        angle_multiplier = 2.0
        left_x, left_y, right_z, roll, pitch, yaw, left_trigger, right_trigger = joy.read()
        
        left_x = int(10 * left_x) / 10.0
        left_y = int(10 * left_y) / 10.0
        right_z = int(10 * right_z) / 10.0
        roll = int(10 * roll) / 10.0
        pitch = int(10 * pitch) / 10.0
        yaw = int(10 * yaw) / 10.0
        left_trigger = left_trigger > 0.5
        right_trigger = right_trigger > 0.5


        left_y = left_y * multiplier
        left_x = left_x * multiplier
        right_z = right_z * multiplier
        roll = roll * angle_multiplier
        pitch = pitch * angle_multiplier
        yaw = yaw * angle_multiplier
        print(left_x, left_y, right_z, roll, pitch, yaw, left_trigger, right_trigger)

        if not left_trigger and not right_trigger:
            joy.robot.move(
                CartesianVelocityMotion(
                    Twist(
                        linear_velocity=[left_x, left_y, right_z], angular_velocity=[roll, pitch, yaw]
                    ),
                    duration=Duration(500),
                ),
                asynchronous=True
            )
        else:
            if left_trigger:
                print("Moving to home")
                time.sleep(2)
                joy.robot.move(home)
                time.sleep(2)
            elif right_trigger:
                print("Resetting to 0 angles")
                time.sleep(2)
                quat = Rotation.from_euler("xyz", [0, 0, 0]).as_quat()
                curr_pose = joy.robot.current_pose
                joy.robot.move(CartesianMotion(RobotPose(Affine(curr_pose.end_effector_pose.translation, quat))))
                time.sleep(2)

        time.sleep(1/100.0)
