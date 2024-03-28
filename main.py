import pybullet as p
import time
from environment.environment import *
from pybullet_planning import plan_rrt_connect

# from grasp_generator import GraspGenerator
import sys
sys.path.append('network')

network_path = 'network/trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch32/epoch_19_iou_0.98'

world = environment()
robot = world.robot

#get camera info from world
camera = world.camera
#create GraspGenerator using world's camera
# generator = GraspGenerator(network_path, camera, 5)

desiredPosition = [1.5, 0.5, 0.5]  # [x, y, z] in meters
desiredOrientation = p.getQuaternionFromEuler([0, 0, 0])  # [roll, pitch, yaw] in radians
desiredPosition2 = [-1.5, -0.5, 0.5]  # [x, y, z] in meters
desiredOrientation2 = p.getQuaternionFromEuler([0, 0, 0])  # [roll, pitch, yaw] in radians

# load objects
objects = YcbObjects('objects/ycb_objects',
                mod_orn=['MediumClamp', 'MustardBottle',
                            'TomatoSoupCan'],
                mod_stiffness=['Strawberry'])
objects.shuffle_objects()
world.obj_init_pos = (0.05, -0.52)




for obj_name in objects.obj_names:
        path, mod_orn, mod_stiffness = objects.get_obj_info(obj_name)
        world.load_isolated_obj(path, mod_orn, mod_stiffness)
        
        rgb, depth, _ = camera.get_cam_img()
        break
        # grasps, save_name = generator.predict_grasp(
        #       rgb, depth, n_grasps = 3, show_output=False
        # )
        # print(obj_name)
        # for i, grasp in enumerate(grasps):
        #     x, y, z, roll, opening_len, obj_height = grasp

        #     print(f'x:{x} y:{y} z:{z} roll:{opening_len} obj_height:{obj_height}')
            
        #     for _ in range(50):
        #         p.stepSimulation() #default stepStimulation delay is 1/240 second
        #     # _, success_target = world.grasp((x,y,z), roll, opening_len, obj_height)

        #     # if success_target: 
        #     #     print('success grasped object')
        #     #     break
        # print('\n')
        # world.remove_all_obj()

# robot.RobotControl.moveToPose(desiredPosition, desiredOrientation)
# robot.RobotControl.openGripper(robot.getJointByName("panda_finger_joint1"),
#                                robot.getJointByName("panda_finger_joint2"))
# while True:
#     pass
# while True:
#     robot.RobotControl.moveToPose(desiredPosition, desiredOrientation)
#     robot.RobotControl.moveToPose(desiredPosition2, desiredOrientation2)




# test

import numpy as np
from scipy.spatial import KDTree
from collections import deque

# PyBullet setup

# Load the robot and obstacles
obstacle1 = p.loadURDF("/home/baotien/panda-arm-pybullet/objects/test_cube1.urdf", [1, 1, 0])
obstacle2 = p.loadURDF("/home/baotien/panda-arm-pybullet/objects/test_cube1.urdf", [-1, -1, 0])
# Start and goal configurations
start_config = np.array([0, 0, 0])
goal_config = np.array([2, 2, 0])

# RRT* motion planner
def rrt_star(start, goal, obstacles, max_iter=1000, step_size=0.1, goal_radius=0.1):
    tree = KDTree([start])
    path = []
    for _ in range(max_iter):
        random_point = np.random.rand(3) * 4 - 2  # Random configuration space
        nearest_idx = tree.query(random_point)[1]
        nearest_point = tree.data[nearest_idx]
        new_point = nearest_point + step_size * (random_point - nearest_point)
        if not collision_check(nearest_point, new_point, obstacles):
            near_indices = tree.query_ball_point(new_point, step_size * 2)
            tree.add(new_point)
            for i in near_indices:
                if not collision_check(tree.data[i], new_point, obstacles):
                    if np.linalg.norm(new_point - goal) < goal_radius:
                        return path_from_parents(tree, i) + [new_point]
                    tree.add(new_point)
                    path.append((tree.data[i], new_point))
    return []

def collision_check(point1, point2, obstacles):
    # Simplified collision check for line segment between two points
    return False  # Implement your collision checking logic here

def path_from_parents(tree, goal_idx):
    path = deque()
    idx = goal_idx
    while idx != 0:
        path.appendleft(tree.data[idx])
        idx = tree.get_parents()[idx]
    return list(path)

# Execute motion planner
path = rrt_star(start_config, goal_config, [obstacle1, obstacle2])

# Execute and visualize the plan
if path:
    for config in path:
        p.resetBasePositionAndOrientation(robot, config, [0, 0, 0, 1])
        time.sleep(0.1)

# Close PyBullet
p.disconnect()

