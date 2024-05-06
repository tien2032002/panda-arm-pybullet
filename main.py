import pybullet as p
import time
from environment.environment import *
import numpy as np
from scipy.spatial import KDTree
from collections import deque
import math
import random
from control import control
from control import motion_plan

# from pybullet_planning import plan_rrt_connect
from grasp_generator import GraspGenerator
import sys
sys.path.append('network')

network_path = 'network/trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch32/epoch_19_iou_0.98'
if __name__ == '__main__':
    world = environment()
    robot = world.robot
    robotControl = control.RobotControl(robot)

    #get camera info from world
    camera = world.camera

    robotControl.move_away_arm()
    generator = GraspGenerator(network_path, camera, 5)
    rgb, depth, _ = camera.get_cam_img()
    grasps, save_name = generator.predict_grasp(
                rgb, depth, n_grasps=3, show_output=False)
    x, y, z, roll, opening_len, obj_height = grasps[0]
    print(grasps[0])
    robotControl.grasp([x, y, z], roll, opening_len, obj_height) 

