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

    #get camera info from world
    camera = world.camera
    #create GraspGenerator using world's camera
    # generator = GraspGenerator(network_path, camera, 5)


        
    # load objects
    # objects = YcbObjects('objects/ycb_objects',
    #                 mod_orn=['MediumClamp', 'MustardBottle',
    #                             'TomatoSoupCan'],
    #                 mod_stiffness=['Strawberry'])
    # objects.shuffle_objects()
    # world.obj_init_pos = (0.05, -0.52)

    #load cup model
    # pose, orientation = world.load_isolated_obj('objects/ycb_objects/YcbCup/model.urdf')
    # print(pose)
    # robot.RobotControl.moveToPose(desiredPosition, desiredOrientation)
    # rgb, depth, _ = camera.get_cam_img()
    # grasps, save_name = generator.predict_grasp(rgb, depth, n_grasps = 3, show_output=True)
    # print(grasps)
    
    # for obj_name in objects.obj_names:
    #     path, mod_orn, mod_stiffness = objects.get_obj_info(obj_name)
    #     world.load_isolated_obj(path, mod_orn, mod_stiffness)
        
    #     rgb, depth, _ = camera.get_cam_img()
        
    #     grasps, save_name = generator.predict_grasp(rgb, depth, n_grasps = 3, show_output=False)
    #     print(obj_name)
    #     for i, grasp in enumerate(grasps):
    #         x, y, z, roll, opening_len, obj_height = grasp

    #         print(f'x:{x} y:{y} z:{z} roll:{opening_len} obj_height:{obj_height}')
            
    #         for _ in range(50):
    #             p.stepSimulation() #default stepStimulation delay is 1/240 second
    #         # _, success_target = world.grasp((x,y,z), roll, opening_len, obj_height)

    #         # if success_target: 
    #         #     print('success grasped object')
    #         #     break
    #     print('\n')
    #     world.remove_all_obj()

    print("do")
    pose = [0.5, 0.5, 0.5]
    orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot.RobotControl.moveToPose(pose, orientation)
    # robot.RobotControl.openGripper(robot.getJointByName("panda_finger_joint1"),
    #                                robot.getJointByName("panda_finger_joint2"))
    # while True:
    #     pass
    # while True:
    #     robot.RobotControl.moveToPose(desiredPosition, desiredOrientation)
    #     robot.RobotControl.moveToPose(desiredPosition2, desiredOrientation2)

    # test
    # Load the robot and obstacles
    # obstacle1 = p.loadURDF("/objects/test_cube1.urdf", [1, 1, 0])
    # obstacle2 = p.loadURDF("/objects/test_cube4.urdf", [-1, -1, 0])
    # Start and goal configurations
    start_config = np.array([0, 0, 0])
    goal_config = np.array([2, 2, 0])

    # Execute motion planner
    # path = rrt_star(start_config, goal_config, [obstacle1, obstacle2])

    # Execute and visualize the plan
    # if path:
    #     for config in path:
    #         p.resetBasePositionAndOrientation(robot, config, [0, 0, 0, 1])
    #         time.sleep(0.1)

#     print (p.getBasePositionAndOrientation())
    while True:
        pass
    # Close PyBullet
    p.disconnect()

