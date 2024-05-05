import pybullet as p
import numpy as np
import math
from control import motion_plan
import random
import time

class RobotControl:
    def __init__(self, robot):
        self.robot = robot
        
        # setup motion plan API
        self.pb_ompl_interface = motion_plan.PbOMPL(self.robot, obstacles=[])
        
        # change planner here
        self.pb_ompl_interface.set_planner("BITstar")
        
        # add obstacles
        # for demo only, rewrite obstacles lib later
        self.obstacles = []
        self.add_obstacles()
        self.gripper_control(0.085)
        
        self.gripper_open_limit = (0.0, 0.1)
        self.ee_position_limit = ((-0.8, 0.8),
                                  (-0.8, 0.8),
                                  (0.785, 1.4))
        
    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        # add box
        self.add_box([1, 0, 0.7], [0.5, 0.5, 0.05])

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id
        

    def moveToPose(self, pose, orientation):
        # calculate ik
        # print(f'current state: {self.robot.get_cur_state()}')
        while (1):
            jointAngles  = p.calculateInverseKinematics(bodyUniqueId = self.robot.id,
                                                        endEffectorLinkIndex = self.robot.end_effector_idx, 
                                                        targetPosition = pose, 
                                                        targetOrientation = orientation,
                                                        lowerLimits = self.robot.lower_limits,
                                                        upperLimits = self.robot.upper_limits)
            for i in range(self.robot.num_dim):
                if jointAngles[i] <= self.robot.lower_limits[i] or jointAngles[i] >= self.robot.upper_limits[i]:
                        continue
            break
        print(f'joint angles: {jointAngles}')
        
        res, path = self.pb_ompl_interface.plan(jointAngles)
        # print (f'path: {path}')
        for state in path: 
            for i in range(self.robot.num_dim):
                pose = state[i]
                j = self.robot.activeJoints[i]
                if j.id in range (9, 19): 
                    continue
                p.setJointMotorControl2(self.robot.id, j.id, p.POSITION_CONTROL,
                                        targetPosition=pose, force=j.maxForce,
                                        maxVelocity=j.maxVelocity)
                p.stepSimulation()
                time.sleep(1/300)
            
    
    def openGripper(self, finger1, finger2):
            pass

    
    def closeGripper(self, finger1, finger2):
        pass
    
    def calc_z_offset(self, gripper_opening_length: float):
        gripper_opening_length = np.clip(
            gripper_opening_length, *self.gripper_open_limit)
        gripper_opening_angle = 0.715 - \
            math.asin((gripper_opening_length - 0.010) / 0.1143)
        gripper_length = 10.3613 * \
            np.sin(1.64534-0.24074 * (gripper_opening_angle / np.pi)) - 10.1219
        return gripper_length
    
    def grasp(self, pos, roll, gripper_opening_length, obj_height):
        x, y, z = pos
        # Substracht gripper finger length from z
        z -= self.finger_length
        z = np.clip(z, *self.ee_position_limit[2])
        self.gripper_control(0.1)
        orn = p.getQuaternionFromEuler([roll, np.pi/2, 0.0])
        self.moveToPose([x, y, 1.25, orn])
        gripper_opening_length *= 0.6
        z_offset = self.calc_z_offset(gripper_opening_length)
        self.moveToPose([x, y, z + z_offset, orn])

    def gripper_control(self, gripper_opening_length):
        # gripper_opening_length = np.clip(gripper_opening_length, *self.gripper_open_limit)
        gripper_opening_angle = 0.715 - math.asin((gripper_opening_length - 0.010) / 0.1143)
        mimic_multiplier = [-1, -1, -1, 1, 1]
        for _ in range(240):
            p.setJointMotorControl2(self.robot.id,
                                self.robot.mimic_parent.id,
                                p.POSITION_CONTROL,
                                targetPosition=gripper_opening_angle,
                                force=self.robot.mimic_parent.maxForce,
                                maxVelocity=self.robot.mimic_parent.maxVelocity)
            
            for i in range(len(self.robot.mimic_children)):
                joint = self.robot.mimic_children[i]
                p.setJointMotorControl2(self.robot.id, joint.id, p.POSITION_CONTROL,
                                    targetPosition=gripper_opening_angle * mimic_multiplier[i],
                                    force=joint.maxForce,
                                    maxVelocity=joint.maxVelocity)
            p.stepSimulation()
            time.sleep(1/240)
       
    def check_grasped(self):
        left_index = self.robot.get_Joint_by_name('left_inner_finger_pad_joint').id
        right_index = self.robot.get_Joint_by_name('right_inner_finger_pad_joint').id

        contact_left = p.getContactPoints(
            bodyA=self.robot_id, linkIndexA=left_index)
        contact_right = p.getContactPoints(
            bodyA=self.robot_id, linkIndexA=right_index)
        contact_ids = set(item[2] for item in contact_left +
                          contact_right if item[2] in [self.obj_id])
        if len(contact_ids) == 1:
            return True
        return False
    
    def pour(self):
        pass

    def stirred(self):
        pass

    def pick(self):
        pass

    def place(self):
        pass