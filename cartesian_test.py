from franky import *
import math
from scipy.spatial.transform import Rotation

robot = Robot("192.168.0.2")
robot.recover_from_errors()

# Reduce the acceleration and velocity dynamic
robot.relative_dynamics_factor = 0.1


quat = Rotation.from_euler("xyz", [0, 0, 0]).as_quat()
curr_pose = robot.current_pose
robot.move(CartesianMotion(RobotPose(Affine(curr_pose.end_effector_pose.translation, quat))))

# # Define and move forwards
# target = Affine([0.0, 0.2, 0.0])
# motion_forward = CartesianMotion(target, reference_type=ReferenceType.Relative)
# robot.move(motion_forward)

# # And move backwards using the inverse motion
# motion_backward = CartesianMotion(
#     target.inverse, reference_type=ReferenceType.Relative
# )
# robot.move(motion_backward)