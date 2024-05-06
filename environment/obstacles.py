import pybullet as p
import pybullet_data
import time
import numpy as np
from environment.camera import Camera
    
class Obstacles:
    SIMULATION_STEP_DELAY = 1 / 240
    center_x, center_y = 0.05, -0.52
    camera = Camera((center_x, center_y, 2.0), (center_x, center_y, 0.785), 0.2, 2.0, (224, 224), 40)

    def __init__(self, camera, vis=False, debug=False) -> None:
        self.vis = vis
        self.debug = debug
        self.camera = camera

        # Create pybullet environment 
        # self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setGravity(0, 0, -10)
        # self.planeID = p.loadURDF('plane.urdf')

        # Create table
        self.table = p.loadURDF("objects/custom_objects/table.urdf", [0.0, -0.65, 0.76], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        self.target_table = p.loadURDF('objects/custom_objects/target_table.urdf', [0.7, 0.0, 0.66], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)

        # Add glass
        # self.cup = p.loadURDF("objects/custom_objects/glass_shelf.urdf", [0, -0.34, 0.88], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        # self.cup1 = p.loadURDF("objects/custom_objects/cup1.urdf", [0.34, -0.75, 0.88], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        # self.cup2 = p.loadURDF("objects/custom_objects/cup2.urdf", [-0.26, -0.58, 0.88], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        # self.cup3 = p.loadURDF("objects/custom_objects/cup3.urdf", [-0.05, -0.8, 0.88], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        # self.cup4 = p.loadURDF("objects/custom_objects/cup4.urdf", [-0.1, -0.42, 0.88], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)

        # Robot arm
        self.arm_table = p.loadURDF("objects/custom_objects/arm_table.urdf", [-0.7, -0.36, 0.0], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        # self.arm = p.loadURDF("objects/custom_objects/ur5_robotiq_140.urdf", [0, 0, 0.0], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

        # Add debug lines for end effector and camera
        if vis:
            self.eef_debug_lineID = None
            p.addUserDebugLine([camera.x, camera.y, 0], [camera.x, camera.y, camera.z], [0, 1, 0], lineWidth=3)
            dist = 1.0
            yaw = 30
            pitch = -50
            target = [0.2, -0.40, 0.785]
            p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

    def step_simulation(self):
        # Hook p.stepSimulation()
        p.stepSimulation()
        # Get camera image
        # rgb, depth, _ = self.camera.get_cam_img()
        if self.vis:
            if self.debug:
                if self.eef_debug_lineID is not None:
                    p.removeUserDebugItem(self.eef_debug_lineID)
                eef_xyz = p.getLinkState(self.arm, 7)[0:1]
                end = np.array(eef_xyz[0])
                end[2] -= 0.5
                self.eef_debug_lineID = p.addUserDebugLine(
                    np.array(eef_xyz[0]), end, [1, 0, 0])
            time.sleep(self.SIMULATION_STEP_DELAY)