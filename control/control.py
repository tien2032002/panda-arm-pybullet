import pybullet as p
import time
import numpy as np
import math

class RobotControl:
    def __init__(self, robot_id, joints, end_effector):
        self.robot_id = robot_id
        self.joints = joints
        self.end_effector = end_effector

    def moveToPose(self, pose, orientation):
        # calculate ik
        jointAngles  = p.calculateInverseKinematics(self.robot_id, self.end_effector.ID, pose, orientation)

        for joint in self.joints:
            print(joint.name + ": " + joint.type)
        for _ in range(300):
            for joint in self.joints:
                if joint.ID <7:
                    pose = jointAngles[joint.ID]
                elif joint.ID == 7:
                    continue
                else:
                    pose = jointAngles[joint.ID - 1]
                p.setJointMotorControl2(self.robot_id, joint.ID, p.POSITION_CONTROL,
                                            targetPosition=pose, force=joint.maxForce,
                                            maxVelocity=joint.maxVelocity)
            p.stepSimulation()
            time.sleep(1/240)

    

    def openGripper(self, finger1, finger2):
        for _ in range(300):
            
            p.setJointMotorControl2(self.robot_id, finger1.ID, p.POSITION_CONTROL,
                                        targetPosition=finger1.upperLimit,
                                        maxVelocity=finger1.maxVelocity)
            p.setJointMotorControl2(self.robot_id, finger2.ID, p.POSITION_CONTROL,
                                        targetPosition=finger2.upperLimit,
                                        maxVelocity=finger2.maxVelocity)
            p.stepSimulation()
            time.sleep(1/240)
    
    def closeGripper(self, finger1, finger2):
        for _ in range(300):
            p.setJointMotorControl2(self.robot_id, finger1.ID, p.POSITION_CONTROL,
                                        targetPosition=finger1.lowerLimit,
                                        maxVelocity=finger1.maxVelocity)
            p.setJointMotorControl2(self.robot_id, finger2.ID, p.POSITION_CONTROL,
                                        targetPosition=finger2.lowerLimit,
                                        maxVelocity=finger2.maxVelocity)
            p.stepSimulation()
            time.sleep(1/240)
    
    
    def grasp(self, pos, roll, gripper_opening_length, obj_height):
        x, y, z = pos
        self.openGripper()
        orn = p.getQuaternionFromEuler([roll, np.pi/2, 0.0])
        self.moveToPose([x - x*math.cos(roll), y - y*math.sin(roll), z], orn)

    def pour(self):
        pass

    def stirred(self):
        pass

    def pick(self):
        pass

    def place(self):
        pass