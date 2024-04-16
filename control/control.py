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
        self.pb_ompl_interface.set_planner("RRT")
        
        # add obstacles
        # for demo only, rewrite obstacles lib later
        self.obstacles = []
        self.add_obstacles()
        
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
        jointAngles  = p.calculateInverseKinematics(bodyUniqueId = self.robot.id,
                                                    endEffectorLinkIndex = self.robot.end_effector.id, 
                                                    targetPosition = pose, 
                                                    targetOrientation = orientation,
                                                    lowerLimits = self.robot.lower_limits,
                                                    upperLimits = self.robot.upper_limits)
        res, path = self.pb_ompl_interface.plan(jointAngles)
        if res:
            for pose in path:
                i = 0
                for joint in self.robot.joints:
                    if joint.type == "FIXED":
                        continue
                    else:
                        jointPose = pose[i]
                    p.setJointMotorControl2(self.robot.id, joint.id, p.POSITION_CONTROL,
                                                targetPosition=jointPose, force=joint.maxForce,
                                                maxVelocity=joint.maxVelocity)
                    i = i + 1
        else:
            print("Invalid initial state, auto generate random state")
            time.sleep
            random_valid_state = [0]*self.robot.num_dim
            for i in range(self.robot.num_dim):
                random_valid_state[i] = self.robot.lower_limits[i] + random.random()*(self.robot.upper_limits[i] - self.robot.lower_limits[i])
            for pose1 in random_valid_state:
                i = 0
                for joint in self.robot.joints:
                    if joint.type == "FIXED":
                        continue
                    else:
                        jointPose = pose1
                    p.setJointMotorControl2(self.robot.id, joint.id, p.POSITION_CONTROL,
                                                targetPosition=jointPose, force=joint.maxForce,
                                                maxVelocity=joint.maxVelocity)
            self.moveToPose(pose, orientation)
                
        

    
    def openGripper(self, finger1, finger2):
            pass

    
    def closeGripper(self, finger1, finger2):
        pass
    
    
    def grasp(self, pos, roll, gripper_opening_length, obj_height):
        pass

    def gripper_control(self, gripper_opening_length):
        # gripper_opening_length = np.clip(gripper_opening_length, *self.gripper_open_limit)
        gripper_opening_angle = 0.715 - math.asin((gripper_opening_length - 0.010) / 0.1143)
        mimic_multiplier = [-1, -1, -1, 1, 1]
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
        
    def pour(self):
        pass

    def stirred(self):
        pass

    def pick(self):
        pass

    def place(self):
        pass