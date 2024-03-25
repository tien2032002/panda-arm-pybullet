import pybullet as p
import time
from control.control import RobotControl

class joint:
    def __init__(self, robot_id, index):
            jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
            self.robot_id = robot_id
            self.info = p.getJointInfo(robot_id, index)
            self.ID = self.info[0]
            self.name = self.info[1].decode("utf-8")
            self.lowerLimit = self.info[8]
            self.upperLimit = self.info[9]
            self.maxForce = self.info[10]
            self.maxVelocity = self.info[11]
            self.type = jointTypeList[self.info[2]]
        
class panda_robot:
    def __init__(self, urdf_path):
        # load urdf
        self.id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        
        #   get robot joints
        self.numJoints = p.getNumJoints(self.id)
        self.joints = [None]*self.numJoints
        for i in range(self.numJoints):
            self.joints[i] = joint(self.id, i)

        # create control api
        self.end_effector = self.joints[7]
        self.RobotControl = RobotControl(self.id, self.joints, self.end_effector)

    
    def getJointByName(self, name):
         for i in range(self.numJoints):
              if self.joints[i].name == name:
                   return self.joints[i]


class environment:
    def __init__(self):
        p.connect(p.GUI)   
        p.resetSimulation()
        # Load robot model
        self.robot = panda_robot("/home/baotien/panda-arm-pybullet/environment/model_description/panda_with_gripper.urdf")
        # load object
        # ...


