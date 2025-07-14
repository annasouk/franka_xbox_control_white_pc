from frankx import Affine, LinearRelativeMotion, Robot, PositionHold, PathMotion, JointMotion

robot = Robot("192.168.0.2")
print(f"Current pose {robot.current_pose()}")
robot.set_default_behavior()
robot.recover_from_errors()
robot.set_dynamic_rel(0.05)
m1 = JointMotion([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
robot.move(m1)