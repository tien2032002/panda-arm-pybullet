import pybullet as p
import numpy as np
import math

class RobotControl:
    def __init__(self, robot):
        self.robot = robot

    def moveToPose(self, pose, orientation):
        # calculate ik
        jointAngles  = p.calculateInverseKinematics(self.robot.id, self.robot.end_effector.id, pose, orientation)
        for _ in range(1):
            i = 0
            for joint in self.robot.joints:
                if joint.type == "FIXED":
                    continue
                else:
                    pose = jointAngles[i]
                p.setJointMotorControl2(self.robot.id, joint.id, p.POSITION_CONTROL,
                                            targetPosition=pose, force=joint.maxForce,
                                            maxVelocity=joint.maxVelocity)
                i = i + 1

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