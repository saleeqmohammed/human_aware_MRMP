# coding=utf-8

"""Synthetic pedestrian behavior with social groups simulation according to the Extended Social Force model.

See Helbing and Molnár 1998 and Moussaïd et al. 2010
"""
from .utils import DefaultConfig
from .scene import PedState, EnvState,RobotState
from . import forces
from .planning import CBMPC
class Simulator:
    """Simulate model.

    ...

    Attributes
    ----------
    state : np.ndarray [n, 6] or [n, 7]
       Each entry represents a pedestrian state, (x, y, v_x, v_y, d_x, d_y, [tau])
    obstacles : np.ndarray
        Environmental obstacles
    groups : List of Lists
        Group members are denoted by their indices in the state
    config : Dict
        Loaded from a toml config file
    max_speeds : np.ndarray
        Maximum speed of pedestrians
    forces : List
        Forces to factor in during navigation

    Methods
    ---------
    capped_velocity(desired_velcity)
        Scale down a desired velocity to its capped speed
    step()
        Make one step
    """

    def __init__(self, state, robot_state, groups=None, obstacles=None, config_file=None):
        self.config = DefaultConfig()
        if config_file:
            self.config.load_config(config_file)
        # TODO: load obstacles from config
        self.scene_config = self.config.sub_config("scene")
        # initiate obstacles
        self.env = EnvState(obstacles, self.config("resolution", 10.0))

        # initiate agents
        self.peds = PedState(state, groups, self.config)
        #initialize robots
        self.robots = RobotState(robot_state,self.config)
        # construct forces
        self.forces = self.make_forces(self.config)

        #setup CB-MPC
        self.mpc = CBMPC(obstacles=obstacles,pos=self.robots.pos(),goal=self.robots.goal(),N=10)

    def make_forces(self, force_configs):
        """Construct forces"""
        force_list = [
            forces.DesiredForce(),
            forces.SocialForce(),
            forces.ObstacleForce(),
            # forces.PedRepulsiveForce(),
            # forces.SpaceRepulsiveForce(),
        ]
        group_forces = [
            forces.GroupCoherenceForceAlt(),
            forces.GroupRepulsiveForce(),
            forces.GroupGazeForceAlt(),
        ]
        if self.scene_config("enable_group"):
            force_list += group_forces

        # initiate forces
        for force in force_list:
            force.init(self, force_configs)

        return force_list

    def compute_forces(self):
        """compute forces"""
        return sum(map(lambda x: x.get_force(), self.forces))

    def get_states(self):
        """Expose whole state"""
        ped_sates,group_states =self.peds.get_states()
        robot_states = self.robots.get_states()
        return  ped_sates,group_states, robot_states

    def get_length(self):
        """Get simulation length"""
        return len(self.get_states()[0])

    def get_obstacles(self):
        return self.env.obstacles

    def step_once(self):
        """step once"""
        self.peds.step(self.compute_forces())

    def calculate_acceleration (self):
        return self.mpc.control_inputs()                   
    def move_robot(self):
        """Move the robot one step"""
        self.robots.step(acc=self.calculate_acceleration())

    def step(self, n=1):
        """Step n time"""
        for _ in range(n):
            self.step_once() #pedestrian step update
            self.move_robot() #robot step updatae
        return self
