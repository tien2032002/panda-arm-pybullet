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
        self.gripper_control(0.085)
        
        self.gripper_open_limit = (0.0, 0.1)
        self.ee_position_limit = ((-0.8, 0.8),
                                  (-0.8, 0.8),
                                  (0.785, 1.4))
        
    def moveToPose(self, pose, orientation):
        # calculate ik
        # print(f'current state: {self.robot.get_cur_state()}')
        while (1):
            jd = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
            0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
            jd = jd * 0
            jointAngles  = p.calculateInverseKinematics(bodyUniqueId = self.robot.id,
                                                        endEffectorLinkIndex = self.robot.end_effector_idx, 
                                                        targetPosition = pose, 
                                                        targetOrientation = orientation,
                                                        jointDamping=jd)
            
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
                time.sleep(1/900)

    def move_away_arm(self):
        joint = self.robot.get_Joint_by_name("shoulder_pan_joint")
        for _ in range(200):
            p.setJointMotorControl2(self.robot.id, joint.id, p.POSITION_CONTROL,
                                    targetPosition=0., force=joint.maxForce,
                                    maxVelocity=joint.maxVelocity)
            p.stepSimulation()
            time.sleep(1/240)
    def auto_close_gripper(self, step: int = 120, check_contact: bool = False) -> bool:
        # Get initial gripper open position
        initial_position = p.getJointState(
            self.robot.id, self.robot.get_Joint_by_name("finger_joint").id)[0]
        initial_position = math.sin(0.715 - initial_position) * 0.1143 + 0.010
        for step_idx in range(1, step):
            current_target_open_length = initial_position - step_idx / step * initial_position

            self.gripper_control(current_target_open_length, 1)
            if current_target_open_length < 1e-5:
                return False

            # time.sleep(1 / 120)
            if check_contact and self.gripper_contact():
                return True
        return False
    
    def gripper_contact(self, bool_operator='and', force=100):
        left_index = self.robot.get_Joint_by_name("left_inner_finger_pad_joint").id
        right_index = self.robot.get_Joint_by_name("right_inner_finger_pad_joint").id


        contact_left = p.getContactPoints(
            bodyA=self.robot.id, linkIndexA=left_index)
        contact_right = p.getContactPoints(
            bodyA=self.robot.id, linkIndexA=right_index)

        if bool_operator == 'and' and not (contact_right and contact_left):
            return False

        # Check the force
        left_force = p.getJointState(self.robot.id, left_index)[
            2][:3]  # 6DOF, Torque is ignored
        right_force = p.getJointState(self.robot.id, right_index)[2][:3]
        left_norm, right_norm = np.linalg.norm(
            left_force), np.linalg.norm(right_force)
        # print(left_norm, right_norm)
        if bool_operator == 'and':
            return left_norm > force and right_norm > force
        else:
            return left_norm > force or right_norm > force

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
        z -= 0.06
        z = np.clip(z, *self.ee_position_limit[2])
        print(f'pose: {x}, {y}, {z}')
        self.gripper_control(0.1)
        orn = p.getQuaternionFromEuler([roll, np.pi/2, 0.0])
        self.moveToPose([x, y, 1.25], orn)
        gripper_opening_length *= 0.6
        z_offset = self.calc_z_offset(gripper_opening_length)
        self.moveToPose([x, y, z + z_offset], orn)
        self.auto_close_gripper(check_contact=True)
        self.moveToPose([x, y, 1.25], orn)
        for _ in range(15):
            p.stepSimulation()
            time.sleep(1 / 300)
            
        
    def moveUp(self, pos, orn):
        x, y, z = pos
        x = np.clip(x, *self.ee_position_limit[0])
        y = np.clip(y, *self.ee_position_limit[1])
        z = np.clip(z, *self.ee_position_limit[2])
        # set damping for robot arm and gripper
        jd = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
              0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        jd = jd * 0
        for _ in range(400):
            # apply IK
            joint_poses = p.calculateInverseKinematics(self.robot.id, self.robot.end_effector_idx, [x, y, z], orn,
                                                       maxNumIterations=100, jointDamping=jd
                                                       )
            # Filter out the gripper
            for i, joint in enumerate(self.robot.activeJoints[:-1]):
                pose = joint_poses[i]
                # control robot end-effector
                p.setJointMotorControl2(self.robot.id, joint.id, p.POSITION_CONTROL,
                                        targetPosition=pose, force=joint.maxForce,
                                        maxVelocity=joint.maxVelocity)

            p.stepSimulation()
            time.sleep(1 / 300)


    def gripper_control(self, gripper_opening_length, step = 120):
        # gripper_opening_length = np.clip(gripper_opening_length, *self.gripper_open_limit)
        gripper_opening_angle = 0.715 - math.asin((gripper_opening_length - 0.010) / 0.1143)
        mimic_multiplier = [-1, -1, -1, 1, 1]
        for _ in range(step):
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