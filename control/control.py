import pybullet as p

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


    def openGripper(self):
        pass
    
    def closeGripper(self):
        pass
    
    def grasp(self):
        pass

    def pour(self):
        pass

    def stirred(self):
        pass

    def pick(self):
        pass

    def place(self):
        pass