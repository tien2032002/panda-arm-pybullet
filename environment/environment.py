import pybullet as p
import time
from control.control import RobotControl
from utils import YcbObjects
import random
import numpy as np
from camera import Camera

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
    TARGET_ZONE_POS = [0.7, 0.0, 0.685]
    def __init__(self):
        self.obj_ids = []
        self.obj_positions = []
        self.obj_orientations = []
        p.connect(p.GUI)   
        p.resetSimulation()
        # Load robot model
        self.robot = panda_robot("environment/model_description/panda_with_gripper.urdf")
        # load tables
        self.tableID = p.loadURDF('environment/urdf/objects/table.urdf',
                                  [0.0, -0.65, 0.76],
                                  p.getQuaternionFromEuler([0, 0, 0]),
                                  useFixedBase=True)
        self.target_table_id = p.loadURDF('environment/urdf/objects/target_table.urdf',
                                          [0.7, 0.0, 0.66],
                                          p.getQuaternionFromEuler([0, 0, 0]),
                                          useFixedBase=True)
        #load camera
        self.camera = Camera((0.05, -0.52, 1.9), (0.05, -0.52, 0.785), 0.2, 2.0, (224, 224), 40)
        


    def load_isolated_obj(self, path, mod_orn=False, mod_stiffness=False):
        r_x = random.uniform(
            self.obj_init_pos[0] - 0.1, self.obj_init_pos[0] + 0.1)
        r_y = random.uniform(
            self.obj_init_pos[1] - 0.1, self.obj_init_pos[1] + 0.1)
        yaw = random.uniform(0, np.pi)

        pos = [r_x, r_y, 0.785]
        obj_id, _, _ = self.load_obj(path, pos, yaw, mod_orn, mod_stiffness)
        for _ in range(10):
           p.stepSimulation()
        self.update_obj_states()
    
    def load_obj(self, path, pos, yaw, mod_orn=False, mod_stiffness=False):
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        obj_id = p.loadURDF(path, pos, orn)
        # adjust position according to height
        aabb = p.getAABB(obj_id, -1)
        if mod_orn:
            minm, maxm = aabb[0][1], aabb[1][1]
            orn = p.getQuaternionFromEuler([0, np.pi*0.5, yaw])
        else:
            minm, maxm = aabb[0][2], aabb[1][2]

        pos[2] += (maxm - minm) / 2
        p.resetBasePositionAndOrientation(obj_id, pos, orn)
        # change dynamics
        if mod_stiffness:
            p.changeDynamics(obj_id,
                             -1, lateralFriction=1,
                             rollingFriction=0.001,
                             spinningFriction=0.002,
                             restitution=0.01,
                             contactStiffness=100000,
                             contactDamping=0.0)
        else:
            p.changeDynamics(obj_id,
                             -1, lateralFriction=1,
                             rollingFriction=0.002,
                             spinningFriction=0.001,
                             restitution=0.01)
        self.obj_ids.append(obj_id)
        self.obj_positions.append(pos)
        self.obj_orientations.append(orn)
        return obj_id, pos, orn
    
    def remove_all_obj(self):
        # self.obj_positions.clear()
        # self.obj_orientations.clear()
        for obj_id in self.obj_ids:
            p.removeBody(obj_id)
        self.obj_ids.clear()
        
    def update_obj_states(self):
        for i, obj_id in enumerate(self.obj_ids):
            pos, orn = p.getBasePositionAndOrientation(obj_id)
            self.obj_positions[i] = pos
            self.obj_orientations[i] = orn

    

