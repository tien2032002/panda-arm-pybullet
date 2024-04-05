import pybullet as p
import time
from environment.environment import *
import numpy as np
from scipy.spatial import KDTree
from collections import deque
import math
import random

# from pybullet_planning import plan_rrt_connect
from grasp_generator import GraspGenerator
import sys
sys.path.append('network')

network_path = 'network/trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch32/epoch_19_iou_0.98'
if __name__ == '__main__':
    world = environment()
    robot = world.robot
    robotControl = RobotControl(robot)

    #get camera info from world
    camera = world.camera
    gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length",
                                            0,
                                            0.085,
                                            0.085)

    # position control
    position_control_group = []
    position_control_group.append(p.addUserDebugParameter('x', -0.5, 0.5, 1))
    position_control_group.append(p.addUserDebugParameter('y', -0.5, 0.5, 1))
    position_control_group.append(p.addUserDebugParameter('z', -0.25, 1, 0.75))
    position_control_group.append(p.addUserDebugParameter('roll', 0, 2*np.pi, np.pi/2))
    position_control_group.append(p.addUserDebugParameter('pitch', 0, 2*np.pi, np.pi/2))
    position_control_group.append(p.addUserDebugParameter('yaw', 0, 2*np.pi, 0))
    while True:
        gripper_opening_length = p.readUserDebugParameter(gripper_opening_length_control)
        # print(f'gripper: {gripper_opening_length}')
        parameter = []
        for i in range(6):
            parameter.append(p.readUserDebugParameter(position_control_group[i]))

        parameter_orientation = p.getQuaternionFromEuler([parameter[3], parameter[4], parameter[5]])
        robotControl.moveToPose(pose=[parameter[0], parameter[1], parameter[2]], orientation=parameter_orientation)
        
        robotControl.gripper_control(gripper_opening_length)
        time.sleep(1/240)
        p.stepSimulation()


