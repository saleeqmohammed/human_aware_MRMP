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
        min_x, max_x, min_y, max_y, grid_size = -12, 12, -12, 12, 1 
        self.grid_env = GridEnvironment(min_x,max_x,min_y,max_y,grid_size)
        #set occuppancy for obstacles
        for obstacle_poses in self.env.obstacles:
            for p_o in obstacle_poses:
                self.grid_env.set_occupancy(p_o[0],p_o[1],True)
        #setup CB-MPC
        self.mpc = CBMPC(obstacles=obstacles,N=10)
        #(obstacles=obstacles,pos=self.robots.pos(),goal=self.robots.goal(),N=10)

        #ROS publisher
        ros_chars = ['/robot_'+str(i) for i in range(7)]
        self.ros_robots =ros_chars[:4]
        self.ros_humans =ros_chars[4:]
        # rospy.init_node('move_peds')
        # self.rate = rospy.Rate(10)
        # self.human_vel_pub = [rospy.Publisher(human+'/cmd_vel',Twist,queue_size=10) for  human in self.ros_humans]
        
    
    def calculate_acceleration (self):
        current_problem = self.mpc.MPC_problem(
            pos=self.robots.pos(),
            goal=self.robots.goal(), 
            )
        
        return self.mpc.control_inputs() 


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
        self.robots.step(acc=self.calculate_acceleration())
 
    def step(self, n=1):
        """Step n time"""
        for _ in range(n):
            self.step_once() #pedestrian step update
            # self.move_robot() #robot step updatae
            # self.rate.sleep()
        return self
    