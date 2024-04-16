import pybullet as p
import pybullet_data
import time
from utils import YcbObjects
import random
import numpy as np
from environment.camera import Camera
import copy

class joint:
    def __init__(self, robot_id, index):
        jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        self.robot_id = robot_id
        self.info = p.getJointInfo(robot_id, index)
        self.id = self.info[0]
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


        #   get robot joints (all of joints, contain fixed joints)
        self.numJoints = p.getNumJoints(self.id)
        
        self.joints = [None] * self.numJoints
        for i in range(self.numJoints):
            self.joints[i] = joint(self.id, i)
            # print(self.joints[i].name)
 
        # dof, only non-fixed joints
        self.activeJoints = [j for j in self.joints if j.type != "FIXED"]
        self.num_dim = len(self.activeJoints)
        
        # joint index
        self.joint_idx = [j.id for j in self.activeJoints]
        
        print (self.joint_idx)

        self.end_effector = self.get_Joint_by_name("panda_robotiq_attachment_joint")
        
        # for gripper
        self.mimic_parent = self.get_Joint_by_name("finger_joint")
        self.mimic_children = [self.get_Joint_by_name("right_outer_knuckle_joint"),
                               self.get_Joint_by_name("left_inner_knuckle_joint"),
                               self.get_Joint_by_name("right_inner_knuckle_joint"),
                               self.get_Joint_by_name("left_inner_finger_joint"),
                               self.get_Joint_by_name("right_inner_finger_joint")]
        
        # add force sensor
        # p.enableJointForceTorqueSensor(self.id, self.get_Joint_by_name("left_inner_finger_pad_joint").id)
        # p.enableJointForceTorqueSensor(self.id, self.get_Joint_by_name("right_inner_finger_pad_joint").id)
        
        # change friction coeff of gripper for better grasp
        # p.changeDynamics(self.id, self.get_Joint_by_name('left_inner_finger_pad_joint').id, lateralFriction=1)
        # p.changeDynamics(self.id, self.get_Joint_by_name('right_inner_finger_pad_joint').id, lateralFriction=1)
        
        self.get_bounds()
        self.reset()
    
    def get_Joint_by_name(self, name):
        for i in range(self.numJoints):
              if self.joints[i].name == name:
                   return self.joints[i]
        print("undefined")
    
    def get_bounds(self):
        self.lower_limits = []
        self.upper_limits = []
        self.joint_bounds = []
        for j in self.activeJoints:
            low = j.lowerLimit # low bounds
            high = j.upperLimit # high bounds
            self.lower_limits.append(low)
            self.upper_limits.append(high)
            if low < high:
                self.joint_bounds.append([low, high])
        print("Joint bounds: {}".format(self.joint_bounds))
        return self.joint_bounds
    
    def get_cur_state(self):
        state = []
        for index in self.joint_idx:
            joint_state = p.getJointState(self.id, index)[0]
            state.append(joint_state)
        self.state = state
        return self.state

    def set_state(self, state):
        '''
        Set robot state.
        To faciliate collision checking
        Args:
            state: list[Float], joint values of robot
        '''
        self._set_joint_positions(self.joint_idx, state)
        self.state = state
        for _ in range(300):
            time.sleep(1/240)
            p.stepSimulation()

    def reset(self):
        '''
        Reset robot state
        Args:
            state: list[Float], joint values of robot
        '''
        # pose = [-1, -1, 1]
        # orientation = p.getQuaternionFromEuler([0, 0, np.pi])
        # print(f'upper limits = {self.upper_limits}')
        # print(f'lower limits = {self.lower_limits}')
        # joint_angles  = p.calculateInverseKinematics(bodyUniqueId = self.id,
        #                                     endEffectorLinkIndex = self.end_effector.id, 
        #                                     targetPosition = pose, 
        #                                     targetOrientation = orientation,
        #                                     lowerLimits = self.lower_limits,
        #                                     upperLimits = self.upper_limits)
        # state = []
        # for i, (lower_limit, upper_limit) in enumerate(self.get_bounds()):
        #     state.append(np.clip(joint_angles[i], lower_limit, upper_limit))
        # self._set_joint_positions(self.joint_idx, state)
        state = [0.01] *self.num_dim
        self.state = state
        self.set_state(state)

    def _set_joint_positions(self, joints, positions):
        for joint, value in zip(joints, positions):
            p.resetJointState(self.id, joint, value, targetVelocity=0)
            

class environment:
    TARGET_ZONE_POS = [0.7, 0.0, 0.685]
    def __init__(self):
        self.obj_ids = []
        self.obj_positions = []
        self.obj_orientations = []
        p.connect(p.GUI)   
        p.resetSimulation()

        # add gravity
        gravity = [0, 0, -9.81]  # Set gravity along the z-axis (negative z direction)
        p.setGravity(gravity[0], gravity[1], gravity[2])

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Load the ground plane
        self.planeid = p.loadURDF("plane.urdf")
        # Load robot model
        self.robot = panda_robot("environment/model_description/urdf/panda.urdf")
        
        # load tables
        # self.tableid = p.loadURDF('environment/urdf/objects/table.urdf',
        #                           [0.0, -0.65, 0.4],
        #                           p.getQuaternionFromEuler([0, 0, 0]),
        #                           useFixedBase=True)
        # self.target_table_id = p.loadURDF('environment/urdf/objects/target_table.urdf',
        #                                   [0.7, 0.0, 0.66],
        #                                   p.getQuaternionFromEuler([0, 0, 0]),
        #                                   useFixedBase=True)
        #load camera

        self.camera = Camera((0.1, -0.4, 1), (-0.033510389871985444, -0.4506058588839027, 0.66), 0.2, 2.0, (224, 224), 40)
        


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
        return p.getBasePositionAndOrientation(obj_id)
    
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
