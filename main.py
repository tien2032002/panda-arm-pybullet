import pybullet as p
import time
from environment.environment import *

panda_world = environment()
robot = panda_world.robot

desiredPosition = [1.5, 0.5, 0.5]  # [x, y, z] in meters
desiredOrientation = p.getQuaternionFromEuler([0, 0, 0])  # [roll, pitch, yaw] in radians
desiredPosition2 = [-1.5, -0.5, 0.5]  # [x, y, z] in meters
desiredOrientation2 = p.getQuaternionFromEuler([0, 0, 0])  # [roll, pitch, yaw] in radians

robot.RobotControl.moveToPose(desiredPosition, desiredOrientation)
robot.RobotControl.openGripper(robot.getJointByName("panda_finger_joint1"),
                               robot.getJointByName("panda_finger_joint2"))
while True:
    pass
# while True:
#     robot.RobotControl.moveToPose(desiredPosition, desiredOrientation)
#     robot.RobotControl.moveToPose(desiredPosition2, desiredOrientation2)
