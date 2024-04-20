try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
    # sys.path.insert(0, join(dirname(abspath(__file__)), '../whole-body-motion-planning/src/ompl/py-bindings'))
    print(sys.path)
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import pybullet as p
from control import collision_check
import time
from itertools import product
import copy

class PbStateSpace(ob.RealVectorStateSpace):
    def __init__(self, num_dim) -> None:
        super().__init__(num_dim)
        self.num_dim = num_dim
        self.state_sampler = None

    def allocStateSampler(self):
        '''
        This will be called by the internal OMPL planner
        '''
        # WARN: This will cause problems if the underlying planner is multi-threaded!!!
        if self.state_sampler:
            return self.state_sampler

        # when ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()

    def set_state_sampler(self, state_sampler):
        '''
        Optional, Set custom state sampler.
        '''
        self.state_sampler = state_sampler
        
class PbOMPL():
    def __init__(self, robot, obstacles = []) -> None:
        '''
        Args
            robot: A PbOMPLRobot instance.
            obstacles: list of obstacle ids. Optional.
        '''
        self.robot = robot
        self.robot_id = robot.id
        self.obstacles = obstacles
        print(self.obstacles)

        self.space = PbStateSpace(robot.num_dim)

        bounds = ob.RealVectorBounds(robot.num_dim)
        joint_bounds = self.robot.get_bounds()
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        self.space.setBounds(bounds)

        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.si = self.ss.getSpaceInformation()
        # self.si.setStateValidityCheckingResolution(0.005)
        # self.collision_fn = pb_utils.get_collision_fn(self.robot_id, self.robot.joint_idx, self.obstacles, [], True, set(),
        #                                                 custom_limits={}, max_distance=0, allow_collision_links=[])

        self.set_obstacles(obstacles)
        
        # change planner here
        self.set_planner("BITstar") # RRT by default

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

        # update collision detection
        self.setup_collision_detection(self.robot, self.obstacles)

    def add_obstacles(self, obstacle_id):
        self.obstacles.append(obstacle_id)

    def remove_obstacles(self, obstacle_id):
        self.obstacles.remove(obstacle_id)
    
    def is_state_valid(self, state):
        # satisfy bounds TODO
        # Should be unecessary if joint bounds is properly set
        # check self-collision
        self.robot.set_state(self.state_to_list(state))
        for link1, link2 in self.check_link_pairs:
            if collision_check.pairwise_link_collision(self.robot_id, link1, self.robot_id, link2):
                # not include collision between eef joints and links
                if link1 in range(9,19) and link2 in range(9,19):
                    continue
                return False
        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if collision_check.pairwise_collision(body1, body2):
                # print('body collision', body1, body2)
                return False
            
        # time.sleep(2)
        return True

    def setup_collision_detection(self, robot, obstacles, self_collisions = True, allow_collision_links = []):
        self.check_link_pairs = collision_check.get_self_link_pairs(robot.id, robot.joint_idx) if self_collisions else []
        moving_links = frozenset(
            [item for item in collision_check.get_moving_links(robot.id, robot.joint_idx) if not item in allow_collision_links])
        moving_bodies = [(robot.id, moving_links)]
        self.check_body_pairs = list(product(moving_bodies, obstacles))

    def set_planner(self, planner_name):
        '''
        Note: Add your planner here!!
        '''
        if planner_name == "PRM":
            self.planner = og.PRM(self.ss.getSpaceInformation())
        elif planner_name == "RRT":
            self.planner = og.RRT(self.ss.getSpaceInformation())
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.ss.getSpaceInformation())
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.ss.getSpaceInformation())
        elif planner_name == "EST":
            self.planner = og.EST(self.ss.getSpaceInformation())
        elif planner_name == "FMT":
            self.planner = og.FMT(self.ss.getSpaceInformation())
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.ss.getSpaceInformation())
        else:
            print("{} not recognized, please add it first".format(planner_name))
            return

        self.ss.setPlanner(self.planner)

    def plan_start_goal(self, start, goal, allowed_time = 0.01):
        '''
        plan a path to gaol from the given robot start state
        '''
        print("start_planning")
        print(self.planner.params())

        orig_robot_state = start
        
        # set the start and goal states;
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]
            print(f'goal: {goal}')

        self.ss.setStartAndGoalStates(s, g)

        # attempt to solve the problem within allowed planning time
        solved = self.ss.solve(allowed_time)
        
        if solved:
            # try to shorten the path
            self.ss.simplifySolution()
            # print the simplified path
            print (self.ss.getSolutionPath())
            
        res = False
        sol_path_list = []
        if solved:
            print("Found solution: interpolating into {} segments".format(300))
            # print the path to screen
            sol_path_geometric = self.ss.getSolutionPath()
            # sol_path_geometric.interpolate(300)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list = [self.state_to_list(state) for state in sol_path_states]
            # print(len(sol_path_list))
            # print(sol_path_list)
            for sol_path in sol_path_list:
                self.is_state_valid(sol_path)
            res = True
        else:
            print("No solution found")

        # reset robot state
        self.robot.set_state(orig_robot_state)
        return res, sol_path_list

    def plan(self, goal, allowed_time = 0.5):
        '''
        plan a path to goal from current robot state
        '''
        start = self.robot.get_cur_state()
        print(f'Start: {start}')
        return self.plan_start_goal(start, goal, allowed_time=allowed_time)

    def execute(self, path, dynamics=True):
        '''
        Execute a planned plan. Will visualize in pybullet.
        Args:
            path: list[state], a list of state
            dynamics: allow dynamic simulation. If dynamics is false, this API will use robot.set_state(),
                      meaning that the simulator will simply reset robot's state WITHOUT any dynamics simulation. Since the
                      path is collision free, this is somewhat acceptable.
        '''
        for q in path:
            if dynamics:
                for i in range(self.robot.num_dim):
                    p.setJointMotorControl2(self.robot.id, i, p.POSITION_CONTROL, q[i],force=5 * 240.)
            else:
                self.robot.set_state(q)
            p.stepSimulation()
            # time.sleep(0.01)
            
    def state_to_list(self, state):
        return [state[i] for i in range(self.robot.num_dim)]