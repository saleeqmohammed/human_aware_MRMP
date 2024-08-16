# coding=utf-8

"""Synthetic pedestrian behavior with social groups simulation according to the Extended Social Force model.

See Helbing and Molnár 1998 and Moussaïd et al. 2010
"""
from .utils import DefaultConfig
from .scene import PedState, EnvState,RobotState
from . import forces
from .planning import CBMPC,GridEnvironment
import rospy
import time
from geometry_msgs.msg import Twist
from .utils.logging import logger
import numpy as np
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

        #descretization for planner
        min_x, max_x, min_y, max_y, grid_size = -15, 15, -15, 15, 0.25 
        self.grid_env = GridEnvironment(min_x,max_x,min_y,max_y,grid_size)
        #set occuppancy for obstacles
        for obstacle_poses in self.env.obstacles:
            for p_o in obstacle_poses:
                self.grid_env.set_occupancy(p_o[0],p_o[1],True)
    
        #inflate obstacles
        self.grid_env.inflate_obstacles(0.32)
        #generate reference paths
        self.reference_paths =[]
        for robot in robot_state:
            start = (robot[0],robot[1])
            goal = (robot[4],robot[5])
            path = self.grid_env.a_star(start,goal)
            self.reference_paths.append(path)
        #setup CB-MPC
        self.mpc = CBMPC(obstacles=obstacles,reference_paths=self.reference_paths,N=30)


        
    
    def calculte_u (self):
        """
        tuple M(Obstacle set, Initial robot positions, Final robot positions, Horizon length) : Problem definition
        """
        N_o = self.get_obstacles() #obstacle set
        X_i = self.robots.get_states()[-1] #inital robot positions for MPC problem is current state
        X_f = self.robots.goal() #final states for robots
        N = 20 #planning horizon
        M =(N_o,X_i,X_f,N)
        node_solution =self.mpc.conflict_solve(M)
        control_inputs = self.mpc.get_velocities(node_solution)
        
        # control_inputs = np.array([
        #     [1,1],
        #     [0,0],
        #     [1,1],
        #     [1,0]
        # ])
        return control_inputs
#robot_0 purple
#robot_1 blue
#robot_2 green
#robot_3 yellow

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
        # velocities = self.peds.vel()
        # for i in range(len(velocities)):
        #     twist = Twist()
        #     twist.linear.x = velocities[i][0]
        #     twist.linear.y = velocities[i][1]
        #     self.human_vel_pub[i].publish(twist)

    def move_robot(self):
        """Move the robot one step"""
        self.robots.step(vel=self.calculte_u())
        #Generete reference paths for robots

 
    def step(self, n=1):
        """Step n time"""
        for _ in range(n):
            self.step_once() #pedestrian step update
            self.move_robot() #Robot step update
        return self