import pybullet as p
import time
from environment.environment import *
from grasp_generator import GraspGenerator
import sys
sys.path.append('network')

network_path = 'network/trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch32/epoch_19_iou_0.98'

world = environment()
robot = world.robot

#get camera info from world
camera = world.camera
#create GraspGenerator using world's camera
generator = GraspGenerator(network_path, camera, 5)

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
        grasps, save_name = generator.predict_grasp(
              rgb, depth, n_grasps = 3, show_output=False
        )
        print(obj_name)
        for i, grasp in enumerate(grasps):
            x, y, z, roll, opening_len, obj_height = grasp

            print(f'x:{x} y:{y} z:{z} roll:{opening_len} obj_height:{obj_height}')
            
            for _ in range(50):
                p.stepSimulation() #default stepStimulation delay is 1/240 second
            # _, success_target = world.grasp((x,y,z), roll, opening_len, obj_height)

            # if success_target: 
            #     print('success grasped object')
            #     break
        print('\n')
        world.remove_all_obj()

# robot.RobotControl.moveToPose(desiredPosition, desiredOrientation)
# robot.RobotControl.openGripper(robot.getJointByName("panda_finger_joint1"),
#                                robot.getJointByName("panda_finger_joint2"))
while True:
    pass
# while True:
#     robot.RobotControl.moveToPose(desiredPosition, desiredOrientation)
#     robot.RobotControl.moveToPose(desiredPosition2, desiredOrientation2)
