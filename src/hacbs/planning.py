#!/usr/bin/env python3
from typing import Tuple
import numpy as np
from pysocialforce.scene import EnvState
import matplotlib.pyplot as plt
from matplotlib import patches
import heapq
from matplotlib.animation import FuncAnimation
import casadi as ca

class GridEnvironment:
    def __init__(self, min_x, max_x, min_y, max_y, grid_size):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.grid_size = grid_size

        self.num_cols = int(np.ceil((max_x - min_x) / grid_size))
        self.num_rows = int(np.ceil((max_y - min_y) / grid_size))

        # Initialize the occupancy grid with False (unoccupied)
        self.occupancy_grid = np.zeros((self.num_rows, self.num_cols), dtype=bool)
    
    def get_cell_center(self, row, col):
        """Get the center coordinates of the grid cell at (row, col)."""
        x_center = self.min_x + (col + 0.5) * self.grid_size
        y_center = self.min_y + (row + 0.5) * self.grid_size
        return (x_center, y_center)
    
    def is_occupied(self, x, y):
        """Check if the cell containing (x, y) is occupied."""
        row, col = self._get_cell_indices(x, y)
        return self.occupancy_grid[row, col]
    
    def set_occupancy(self, x, y, occupied=True):
        """Set the occupancy of the cell containing (x, y)."""
        row, col = self._get_cell_indices(x, y)
        self.occupancy_grid[row, col] = occupied
    def is_within_bounds(self,x,y):
        is_within_bounds= True
        if x <= self.min_x or x>= self.max_x:
            is_within_bounds = False
        if y<= self.min_y or y>=self.max_y:
            is_within_bounds = False
        return is_within_bounds
    
    def block_range(self, center_x, center_y, radius):
        """Block cells within a given radius around a center point (center_x, center_y)."""
        center_row, center_col = self._get_cell_indices(center_x, center_y)
        inflation_cells = int(np.ceil(radius / self.grid_size))

        for row in range(center_row - inflation_cells, center_row + inflation_cells + 1):
            for col in range(center_col - inflation_cells, center_col + inflation_cells + 1):
                if 0 <= row < self.num_rows and 0 <= col < self.num_cols:
                    # Calculate the actual distance from the center in grid units
                    cell_center_x, cell_center_y = self.get_cell_center(row, col)
                    if np.linalg.norm(np.array([cell_center_x, cell_center_y]) - np.array([center_x, center_y])) <= radius:
                        self.set_occupancy(cell_center_x, cell_center_y, True)

    def _get_cell_indices(self, x, y):
        """Convert (x, y) coordinates to grid cell indices."""
        col = int((x - self.min_x) // self.grid_size)
        row = int((y - self.min_y) // self.grid_size)
        return (row, col)
    def inflate_obstacles(self, inflation_radius):
        """Inflate obstacles in the occupancy grid by a given radius."""
        inflated_grid = self.occupancy_grid.copy()

        # Convert the inflation radius from meters to grid cells
        inflation_cells = int(np.ceil(inflation_radius / self.grid_size))

        for row in range(self.num_rows):
            for col in range(self.num_cols):
                if self.occupancy_grid[row, col]:  # If this cell is occupied
                    # Inflate surrounding cells within the inflation radius
                    for i in range(-inflation_cells, inflation_cells + 1):
                        for j in range(-inflation_cells, inflation_cells + 1):
                            new_row = row + i
                            new_col = col + j
                            # Check if the new cell is within the grid bounds
                            if 0 <= new_row < self.num_rows and 0 <= new_col < self.num_cols:
                                inflated_grid[new_row, new_col] = True
        
        self.occupancy_grid = inflated_grid
    def print_grid(self):
        """Print the occupancy grid for debugging."""
        print(self.occupancy_grid)

    def heuristic(self, a, b):
        """Heuristic function for A* (Manhattan distance)."""
        # return abs(a[0] - b[0]) + abs(a[1] - b[1])
        """Heuristic function for A* (Euclidean distance)."""
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def a_star(self, start, goal):
        """A* pathfinding algorithm."""
        start_indices = self._get_cell_indices(*start)
        goal_indices = self._get_cell_indices(*goal)

        if not (0 <= start_indices[0] < self.num_rows and 0 <= start_indices[1] < self.num_cols):
            raise ValueError("Start position is out of the grid bounds.")
        if not (0 <= goal_indices[0] < self.num_rows and 0 <= goal_indices[1] < self.num_cols):
            raise ValueError("Goal position is out of the grid bounds.")

        open_set = []
        heapq.heappush(open_set, (0, start_indices))
        came_from = {}
        g_score = {start_indices: 0}
        f_score = {start_indices: self.heuristic(self.get_cell_center(*start_indices), goal)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal_indices:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.get_neighbors(current):
                # Determine movement cost
                if (current[0] != neighbor[0] and current[1] != neighbor[1]):
                    # Diagonal movement
                    tentative_g_score = g_score[current] + np.sqrt(2) * self.grid_size
                else:
                    # Horizontal or vertical movement
                    tentative_g_score = g_score[current] + self.grid_size
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(self.get_cell_center(*neighbor), goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # No path found

    def get_neighbors(self, indices):
        """Get neighboring cells (up, down, left, right, and diagonals) of the current cell."""
        row, col = indices
        neighbors = []
        
        # 8 possible moves: vertical, horizontal, and diagonal
        for d_row, d_col in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            new_row = row + d_row
            new_col = col + d_col
            if 0 <= new_row < self.num_rows and 0 <= new_col < self.num_cols:
                if not self.occupancy_grid[new_row, new_col]:
                    neighbors.append((new_row, new_col))
        
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        # Convert path from grid indices to actual coordinates
        return [self.get_cell_center(row, col) for row, col in path]



    
class Node:
    def __init__(self,epsilon_r,epsilon_o,D):
        self.cost = None
        self.solution = {}
        self.constraints={}
        self.obstacles={}
        self.humans =[]
        self.epsilon_r = epsilon_r
        self.epsilon_o = epsilon_o
        self.D = 1.0#D
        self.HSA = 1.0
    def check_robot_collision(self,trajectory_i,trajectory_j,delta_r):
        """
        Constriant 3:
        inter robot collision
        ||P_j^k - P_i^k|| >= D + delta_r + epsilon_r
        """
        bool_filter =np.linalg.norm(trajectory_i-trajectory_j,axis=1) >= self.D + delta_r + self.epsilon_r
        r_collision_tidx = np.where(bool_filter==False)[0]
        return r_collision_tidx
    def check_obstacle_collision(self,obstacles,trajectory,delta_o):
        """
        Check collison between obstacles and agent
        Input: obstacles,trajectory,delta_o,epsilon_o

        Constraint 4
        ||P_p^o - P_i^k|| >= D/2+ delta_o + epsilon_o

        output: status, o_collision_index
        """
        o_collision_tidx =[]
        o_idx=[]
        if len(obstacles):
            difference_matrix =  trajectory[:,np.newaxis,:2] - obstacles[np.newaxis,:,:]
            distance_matrix = np.linalg.norm(difference_matrix,axis=2)
            bool_filter = distance_matrix >= self.D/2 + self.epsilon_o + delta_o 
            o_collision_tidx,o_idx = np.where(bool_filter==False)
        
        return o_collision_tidx, o_idx
    def check_human_collision(self,trajectory,humans,delta_h):
        """
        Check collision between humans and agent
        Input: trajectory of agent, human positions, delta_h slack
        """
        h_collision_tidx = []
        h_idx = []
        difference_matrix = trajectory[:,np.newaxis,:2] - humans[np.newaxis,:,:]
        distance_matrix = np.linalg.norm(difference_matrix,axis=2)
        bool_filter = distance_matrix >= self.HSA + delta_h
        h_collision_tidx,h_idx =np.where(bool_filter==False)
        return h_collision_tidx,h_idx
    def conflicts(self):
        """  
        Return the list of conflicts in given solution node
        Check collisions with respect to constraints 3,4 in Tajbaksh et al.
        """
        conflicts =[]
        N_a = len(self.solution.keys())
        for a_i in range(N_a):
            #check obstacle collision for each agent
            x_solution_i,_,delta_r_i,delta_o_i,delta_h_i = self.solution[a_i]
            o_collision_tidx,o_collision_oidx =self.check_obstacle_collision(self.obstacles[a_i],x_solution_i,delta_o_i)
            #if there is collision add conflict
            if len(o_collision_tidx):
                t_c = o_collision_tidx[0]
                t_h = len(x_solution_i)
                conflicts.append((a_i,-1,[t_c,t_h]))
            #check collision with every other robot
            for a_j in range(a_i+1,N_a):
                x_solution_j,_,delta_r_j,delta_o_j,delta_h_j = self.solution[a_j]
                delta_r = max(delta_r_i,delta_r_j)
                r_collision_tidx =self.check_robot_collision(x_solution_i,x_solution_j,delta_r)
                #if there is collision add conflict
                if len(r_collision_tidx):
                    t_c = r_collision_tidx[0]
                    t_h = len(x_solution_i)
                    conflicts.append((a_i,a_j,[t_c,t_h]))
            #check collision with humans
            if len(self.humans):
                humans = self.humans[:,:2]
                h_collision_tidx,h_collision_h_idx = self.check_human_collision(x_solution_i,humans,delta_h_i)
                #if there is collision add conflict
                if len(h_collision_tidx):
                    t_c = h_collision_tidx[0]
                    t_h = len(x_solution_i)
                    conflicts.append((a_i,-2,[t_c,t_h]))
        #sort conflicts with earliest tc first
        conflicts_sorted = sorted(conflicts,key=lambda x: x[2][0])
        return conflicts_sorted



class CBMPC:
    def __init__(self,obstacles,reference_paths,N):
        self.obstacles = obstacles
        self.N = N 
        self.agents =0
        self.reference_paths=reference_paths
        # Define weights
        self.Q =3*np.diag([12.0, 12.0]) # Weight for state tracking error
        self.R = 8*np.diag([12.0, 12.0])       # Weight for control effort
        self.P_term = np.diag([12.5,12.5])  # Weight for goal tracking error
        #constrint parameters
        self.epsilon_g = 0.02 #goal tolerance
        self.D = 1.04 #robot footprint
        self.HSA = 1.1 #human safety area
        self.epsilon_r = 0.08 #robot robot collision tolerance   
        self.kr = 10e6 #inter robot collision tolerance slcak coefficient
        self.ko = 10e18 #obstacle collision tolerance slack coefficient
        self.kh = 10e8 #human robot collision tolereance slack coefficient
        self.epsilon_o = 0.05#robot-obstacle collision tolerance
        self.epsilon_hsa = 0.05 #human robot collision tolerance
        #mpc parameters
        self.T = 0.2 #mpc time step
        self.nx = 2 #number of states [x,y]
        self.nu = 2 #number of controls [v_x,v_y]

        #create open list for solution
        self.o_list = []


    @staticmethod
    def trajectory_length(solution):
        # Initialize the total length to zero
        total_length = 0.0

        # Iterate over the points in the trajectory
        for i in range(1, len(solution)):
            # Get the previous and current point
            prev_point = solution[i - 1]
            current_point = solution[i]

            # Calculate the Euclidean distance between the points
            distance = np.linalg.norm(current_point - prev_point)

            # Add the distance to the total length
            total_length += distance

        return total_length
    @staticmethod
    def precompute_cumulative_distances(path):
        """Precompute cumulative distances along the path."""
        cumulative_distances = [0]  # Start at 0 for the first point
        for i in range(1, len(path)):
            dist = np.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            cumulative_distances.append(cumulative_distances[-1] + dist)
        return cumulative_distances
    @staticmethod
    def find_closest_point(precomputed_distances, path, coordinate):
        """Find the closest point in the path to the given coordinate."""
        print(f"coordinate: {coordinate}")
        min_distance = float('inf')
        closest_index = 0
        for i, (x, y) in enumerate(path):
            distance = np.hypot(x - coordinate[0], y - coordinate[1])
            print()
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        return closest_index
    @staticmethod
    def find_point_at_distance(precomputed_distances, start_index, distance):
        """Find the point in the path at a specific distance from the start_index."""
        target_distance = precomputed_distances[start_index] + distance
        for i in range(start_index, len(precomputed_distances)):
            if precomputed_distances[i] >= target_distance:
                return i
        return len(precomputed_distances) - 1  # Return the last point if the distance exceeds the path length

    def get_obstacles_in_range(self,pos,obstacles)->np.ndarray:
        """
        Get obstacles in range of horizon
        Input: position of robot, all obstacles in the environment
        Output: list of obstacles in range
        """
        obstacles_inrange=[]
        for obstacle in obstacles:
            obstacle_dists = np.linalg.norm(obstacle-pos[:2],axis=1)
            min_dist_idx =np.argmin(obstacle_dists)
            if obstacle_dists[min_dist_idx] < 5:
                # print(f"Added obstacle point {obstacle[min_dist_idx]} at distance: {obstacle_dists[min_dist_idx]}")
                #we will take only the closest point in the obstacle to plan for collision
                obstacles_inrange.append(obstacle[min_dist_idx].tolist())
                obstacles_inrange.append(obstacle[0].tolist())
                obstacles_inrange.append(obstacle[-1].tolist())
                #add some more points from the same obstacle
                obstacle_length = obstacle.shape[0]
                sample_length = obstacle_length//10
                sample_indeces = np.random.choice(obstacle.shape[0],size=sample_length,replace=False)
                sample_array= obstacle[sample_indeces]
                obstacles_inrange.extend(sample_array.tolist())
        return np.array(obstacles_inrange)
    def generate_reference(self,path, coordinate, distance, N, precomputed_distances=None):
        """
        Generate a linspace of size N for a section of the path, including x, y, and theta.
        - x, y: Coordinates of the points.
        - theta: Angle between consecutive points.

        Parameters:
        - path: List of tuples, where each tuple is an (x, y) pair.
        - coordinate: Tuple (x, y), the reference coordinate to find the closest point in the path.
        - distance: The distance from the closest point to find the end point in the path.
        - N: Number of points in the resulting linspace, including the endpoints.
        - precomputed_distances: Optional list of precomputed cumulative distances along the path.

        Returns:
        - trajectory: Array of shape (N, 3), each row representing (x, y, theta).
        """
      
        if precomputed_distances is None:
            precomputed_distances = self.precompute_cumulative_distances(path)
        
        # Find the closest point to the given coordinate
        start_index = self.find_closest_point(precomputed_distances, path, coordinate)
        
        # Find the point at the given distance from the closest point
        end_index = self.find_point_at_distance(precomputed_distances, start_index, distance)

        if start_index == end_index:
            start_index = end_index - 1
        # Extract the relevant section of the path
        section = path[start_index:end_index + 1]
        section_distances = precomputed_distances[start_index:end_index + 1]

        # Normalize distances to [0, 1] for interpolation
        normalized_distances = (np.array(section_distances) - section_distances[0]) / (section_distances[-1] - section_distances[0])

        # Generate linspace of size N over the range [0, 1]
        linspace = np.linspace(0, 1, N)

        # Separate x and y coordinates
        x_coords = [point[0] for point in section]
        y_coords = [point[1] for point in section]

        # Interpolate x and y coordinates along the linspace
        linspace_x = np.interp(linspace, normalized_distances, x_coords)
        linspace_y = np.interp(linspace, normalized_distances, y_coords)

        

        # Combine interpolated x, y coordinates, and theta into (x, y, theta) format
        trajectory = np.column_stack((linspace_x, linspace_y))

        return trajectory

    #method to add a node in olist
    def insert_node(self,R:Node):
        """
        Inserts node R into list o_list maintaining ascending order based on R.cost.
        """
        # Perform binary search manually to find the correct insertion point
        lo, hi = 0, len(self.o_list)
        while lo < hi:
            mid = (lo + hi) // 2
            if self.o_list[mid].cost < R.cost:
                lo = mid + 1
            else:
                hi = mid
        self.o_list.insert(lo, R)
    
    def MPC(self,M,a_i,Constarints)->list:
        """
        Model Predictive control subroutine
        MPC(M,a_i,C)
        Inputs :-
        M : Problem definition
        a_i : Agent index
        C = A.constraints : set of "agent specific" constraints of the form (a_i, a_j,[t_c1,t_c2]) for all a_i != a_j
        Output :-
        J_i : cost of solution for agent i
        T_i = Trajectory of agent i
        """
        N= self.N
        T = self.T
        nu = self.nu
        nx = self.nx
        Q = self.Q
        R = self.R
        P_term = self.P_term
        kr = self.kr
        ko = self.ko
        kh = self.kh
        #Define symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
    
        states = ca.vertcat(x, y)
        v_x = ca.SX.sym('v_x')
        v_y= ca.SX.sym('v_y')
        controls = ca.vertcat(v_x,v_y)
        delta_r = ca.SX.sym('delta_r')
        delta_o = ca.SX.sym('delta_o')
        delta_hsa = ca.SX.sym('delta_hsa')
        # System dynamics
        rhs = ca.vertcat(v_x,v_y)

        # Define the function f(x,u)
        f = ca.Function('f', [states, controls], [rhs])

        # Define optimization variables
        U = ca.SX.sym('U', nu, N)  # Control inputs for N steps
        P = ca.SX.sym('P', nx * (N + 2))  # Reference states for N+1 steps + goal state
        X = ca.SX.sym('X', nx, N+1)  # States for N+1 steps

        # Objective function and constraints
        obj = 0  # Objective function
        g = []   # Constraints

        # Define trajectory reference states
        ref_trajectory = P[:nx * (N+1)].reshape((nx, N+1))  # Reference trajectory over N+1 steps
        goal_state = P[nx * (N+1):]  # Goal state of the robot in global reference

       # Initial state constraint 
        """
        Constraint 6 Tajbakhsh et al.

        """
        st = X[:, 0]
        g.append(st - ref_trajectory[:, 0]) 
        # Formulate the optimization problem with weights
        for k in range(N):
            st = X[:, k]
            con = U[:, k]
            # Cost function: weighted state deviation from trajectory + weighted control effort + terminal cost
            obj += ca.mtimes([(st - ref_trajectory[:, k]).T, Q, (st - ref_trajectory[:, k])]) + ca.mtimes([con.T, R, con]) + kr*delta_r + ko*delta_o + kh*delta_hsa +ca.mtimes([(X[:,N]-goal_state).T,P_term,(X[:,N]-goal_state)]) #ca.mtimes([(X[:,N]).T,P_term,(X[:,N])])
            st_next = X[:, k+1]
            f_value = f(st, con)
            st_next_euler = st + T*f_value
            #constraint 1 in tajbaksh et al.
            g.append(st_next - st_next_euler)
     

        # Define the bounds
        lbx = []
        ubx = []

        # State bounds
        for _ in range(N + 1):
            lbx.extend([-ca.inf, -ca.inf])
            ubx.extend([ca.inf, ca.inf])

        # Control bounds
        for _ in range(N):
            lbx.extend([-1.0, -1.0])  # vx lower bound, vy lower bound
            ubx.extend([1.0, 1])    # v upper bound, omega upper bound

        # Obstacle constraint bounds
        lbx.extend([0.0])  # Lower bound for delta_r
        ubx.extend([ca.inf])  # Upper bound for delta_r
        lbx.extend([0.0])  # Lower bound for delta_o
        ubx.extend([ca.inf])  # Upper bound for delta_o
        lbx.extend([0.0]) #Lower bound for delta_hsa
        ubx.extend([ca.inf])# Upper bound for delta_hsa
        lbx = ca.vertcat(*lbx)
        ubx = ca.vertcat(*ubx)

        # Constraint bounds (equality constraints)
        lbg = [0] * (nx * (N + 1))  # Constraints for state and control constraints
        ubg = lbg.copy()


        # Additional collision constraints and bounds
        for C in Constarints: 
        #if obstacles (1:obstacles, 0: robots)
            if C[0]==1:
                #static obstacles
                t_c, t_h = C[2]
                obs = C[1]
                N_o = len(obs)
                # Define obstacle positions in symbolic variables
                O_sym = ca.SX.sym('O_sym', 2 * N_o)  # Adjust size to match the number of obstacles
                O_pos = O_sym.reshape((2, N_o))

                for l in range(t_c, t_h):
                    for o_idx in range(N_o):
                        st_pos = X[:2, l]
                        p_o = O_pos[:, o_idx]
                        distance_squared = ca.sumsqr(p_o - st_pos)
                        rhs_lim = ((self.D/2) + delta_o + self.epsilon_o) ** 2
                        # Add constraint
                        g.append(rhs_lim - distance_squared)
                        # Add bounds
                        lbg.append(-ca.inf)
                        ubg.append(0)
                
                # Add obstacle parameters to P
                P = ca.vertcat(P, O_sym)
            if C[0] == 0:
                #robot collision
                t_c,t_h = C[2]
                #we need all xsolutions from tc to th
                #at each of the times from tc to th the robot should be at constraint distance
                x_j =C[1]
                N_r = len(x_j)
                R_sym = ca.SX.sym('R_sym',2*N_r)
                R_pos = R_sym.reshape((2,N_r))
                for l in range(t_c,t_h):
                    #here the position of other robot p_r can be occuppied by the agent at some other time tl so no need to restric it with N_r X l constraiints
                    st_pos = X[:2,l]
                    p_r = R_pos[:2,l]
                    distance_squared = ca.sumsqr(p_r-st_pos)
                    rhs_lim = (self.D + delta_r + self.epsilon_r)**2
                    #add constraint
                    g.append(rhs_lim - distance_squared)
                    #add bounds
                    lbg.append(-ca.inf)
                    ubg.append(0)
                P = ca.vertcat(P,R_sym)
            if C[0] == 2:
                #human collision
                t_c,t_h =C[2]
                humans = C[1]
                N_h = len(humans)
                #Define human positons in symbolic variables
                H_sym = ca.SX.sym('H_sym',2*N_h)
                H_pos = H_sym.reshape((2,N_h))
                for l in range(t_c,t_h):
                    for h_idx in range(N_h):
                        st_pos = X[:2, l]
                        p_h = H_pos[:, h_idx]
                        distance_squared = ca.sumsqr(p_h - st_pos)
                        rhs_lim = ((self.HSA + delta_hsa)) ** 2
                        # Add constraint
                        g.append(rhs_lim - distance_squared)
                        # Add bounds
                        lbg.append(-ca.inf)
                        ubg.append(0)
                
                # Add obstacle parameters to P
                P = ca.vertcat(P, H_sym)
        # Finalize bounds
        lbg = ca.vertcat(*lbg)
        ubg = ca.vertcat(*ubg)

        # Define optimization variables
        OPT_variables = ca.vertcat(ca.reshape(X, nx * (N + 1), 1), ca.reshape(U, nu * N, 1), delta_r, delta_o,delta_hsa)

        # Define NLP problem
        nlp_prob = {
            'f': obj,
            'x': OPT_variables,
            'g': ca.vertcat(*g),
            'p': P
        }

        # Solver options
        opts = {
            'ipopt': {
                'max_iter': 200,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            }
        }
        # Create solver
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        # Initial state and trajectory reference states
        robot_state = np.squeeze(M[1])
        pos = robot_state[a_i]
        
        trajectory = self.generate_reference(self.reference_paths[a_i], pos, 5, self.N + 1)
        goal = trajectory[-1]
        #add the defarult paraams for trajectory and goal
        p = np.concatenate((trajectory.flatten(), goal))

        for C in Constarints:
            if C[0]==1:
                r_obstacles = C[1]
                p = np.concatenate((p,r_obstacles.flatten()))
            if C[0]==0:
                x_sol_j = C[1]
                p = np.concatenate((p,x_sol_j.flatten()))
            if C[0]==2:
                p_humans = C[1]
                p = np.concatenate((p,p_humans.flatten()))
        # Initial guess
        x_init = np.zeros((nx * (N + 1) + nu * N + 3, 1))
        # Solve the problem
        sol = solver(x0=x_init, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

        # Extract solution
        sol_x = sol['x'].full().flatten()
        x_solution = sol_x[:nx * (N + 1)].reshape((N + 1, nx))
        u_solution = sol_x[nx * (N + 1):-3].reshape((N, nu))
        delta_r_solution = sol_x[-3]  # Extract delta_r
        delta_o_solution = sol_x[-2]  # Extract delta_o
        delta_h_solution = sol_x[-1] #TODO: Extract delta_h



        # # Plot setup
        # for C in Constarints:
        #     # Convert ref_trajectory to numerical format
        #     ref_trajectory_numerical = trajectory.T  # Transpose to match shape (nx, N+1)

        # # Plotting and animation
        #     def animate(i):
        #         plt.clf()
        #         plt.plot(ref_trajectory_numerical[0, :], ref_trajectory_numerical[1, :], 'g--', label='Reference Trajectory')
        #         plt.plot(x_solution[:, 0], x_solution[:, 1], 'b-', label='Agent Path')
        #         plt.scatter(x_solution[i, 0], x_solution[i, 1], color='red')  # Current position of the agent
        #         for C in Constarints:
        #             r_obstacles = C[1]
        #             plt.scatter(r_obstacles[:,0],r_obstacles[:,1],color='black')
                    
        #         plt.xlim(min(ref_trajectory_numerical[0, :])-1, max(ref_trajectory_numerical[0, :])+1)
        #         plt.ylim(min(ref_trajectory_numerical[1, :])-1, max(ref_trajectory_numerical[1, :])+1)
        #         plt.xlabel('x')
        #         plt.ylabel('y')
        #         plt.title('Agent Path and Reference Trajectory')
        #         plt.legend()

        #     fig = plt.figure()
        #     ani = FuncAnimation(fig, animate, frames=N+1, interval=200, repeat=True)

        #     ani.save(f'animation{a_i}.gif',writer='pillow',fps=5)
        #     plt.show()

        #Return solutions
        return [x_solution, u_solution, delta_r_solution, delta_o_solution,delta_h_solution]


    def SIC(self,solution,goals):
        """
        Method to calculate sum of induvidual costs
        J_n = J_c + J_f

        Input: set of x_i in solution
        """
        J = 0
        for n in range(self.agents):

            sol_n= solution[n]
            solution_trajectory =sol_n[1]
            # print(f"solution trajectory {solution_trajectory}")
            #trajectory length till now
            J_c = self.trajectory_length(solution_trajectory)
            #distance left
            goal = goals[n]
            endpoint = solution_trajectory[-1]
            J_f = np.linalg.norm(goal - endpoint)
            J_n = J_c + J_f
            J += J_n
        return J
    def conflict_solve(self,M):
        """
        Input: M(N_o,X_i,X_f,N)
        Output: Conllision Free trajectories P.solution
        """
        self.agents = len(np.squeeze(M[1]))
        goals = np.squeeze(M[2]) #X_f
        print(f"number of agents:{self.agents}")
        R = Node(self.epsilon_r,self.epsilon_o,self.D)
        R.humans = np.squeeze(M[3])
        for a_i in range(self.agents):
            print(f"Solving for agent: {a_i}")
            #Find the obstacles in planned MPC horizon
            #get the agent location from M
            robot_state = np.squeeze(M[1])
            pos = robot_state[a_i]
            #get obstacles set from M
            obs_h = self.get_obstacles_in_range(pos,M[0])
            R.obstacles[a_i] =obs_h
            #get humans from M
            #solve the MPC problem
            R.solution[a_i] = self.MPC(M,a_i,{}) #No additional constraints initially
        R.cost =self.SIC(R.solution,goals) #calculate SIC for solution trajectory
        print(f"R.cost: {R.cost}")

        self.insert_node(R)

        while(True):
            P = self.o_list.pop(0)
            if len(P.conflicts()) ==0:
                return P.solution
            C = P.conflicts()[0] #first conflict
            a_i,a_j = C[0],C[1]

            #deal with obstacle collision
            if a_j == -1:
                A = Node(self.epsilon_r,self.epsilon_o,self.D)

                #start with precalculated solutions
                A.solution = P.solution.copy()
                #get pre assigned obstacles
                A.obstacles = P.obstacles.copy()
                A.constraints = P.constraints.copy()
                #add the new constraints
                #constraint format (constraint_type=1( 1: obstacle, 0: robot),obstacles,[tc,th])
                try:
                    A.constraints[a_i].append((1,A.obstacles[a_i],C[2]))
                except:
                    A.constraints[a_i] = [(1,A.obstacles[a_i],C[2])]
                A.solution[a_i] = self.MPC(M,a_i,A.constraints[a_i])
                A.cost = self.SIC(A.solution,goals)
                self.insert_node(A)
            elif a_j== -2:
                #deal with human collision
                A = Node(self.epsilon_r,self.epsilon_o,self.D)

                #start with precalculated solutions
                A.solution = P.solution.copy()
                #get pre assigned obstacles
                A.obstacles = P.obstacles.copy()
                #get humans
                A.humans = P.humans.copy()
                #Copy constraints
                A.constraints = P.constraints.copy()
                #add the new constraints
                #constraint format (constraint_type=1( 1: obstacle, 0: robot,  2:human),obstacles,[tc,th])
                try:
                    A.constraints[a_i].append((2,A.humans[:,:2],C[2]))
                except:
                    A.constraints[a_i] = [(2,A.humans[:,:2],C[2])]
                A.solution[a_i] = self.MPC(M,a_i,A.constraints[a_i])
                A.cost = self.SIC(A.solution,goals)
                self.insert_node(A)
            else:
                #deal with robot-robot collision
                C_agents = [a_i,a_j]
                for i in range(len(C_agents)):
                    a_i = C_agents[i]
                    A = Node(self.epsilon_r,self.epsilon_o,self.D)
                    A.solution = P.solution.copy()
                    A.obstacles = P.obstacles.copy()
                    A.constraints = P.constraints.copy()
                    #to generate constraints, we need to get the x_solution for the second robot 
                    a_j = C_agents[i-1] #always selects the other sice there are only 2 elements
                    x_solution_aj,_,_,_,_ = P.solution[a_j].copy()
                    try:
                        A.constraints[a_i].append((0,x_solution_aj,C[2]))
                    except:
                        A.constraints[a_i] = [(0,x_solution_aj,C[2])]
                    A.solution[a_i] = self.MPC(M,a_i,A.constraints[a_i])
                    A.cost = self.SIC(A.solution,goals) 
                    self.insert_node(A)

    @staticmethod
    def get_velocities(solution: dict) -> np.ndarray:
        """
        Extracts the first velocity components (vx, vy) from the control inputs
        provided in the solution dictionary.
        
        Parameters:
        - solution: A dictionary where keys are agent indices and values are tuples containing 
                    (state_trajectory, control_inputs). Control inputs are in the form [v, omega].
        
        Returns:
        - A NumPy array where each row contains [vx, vy] for each agent at the first time step.
        """
        velocities = []
        
        for a_i in solution.keys():
            # Extract state trajectory and control inputs for the agent
            _, u_solution,_,_ ,_= solution[a_i]
            
            # Only use the first control input
            if len(u_solution) > 0:
                # Extract the first control input
                v_x = u_solution[0, 0]  # Linear velocity
                v_y = u_solution[0, 1]  # Angular velocity (not used directly here)
                
                # Extract the initial orientation theta from the state trajectory
                
                velocities.append([v_x, v_y])
        
        # Convert list of arrays into a single NumPy array
        velocity_vectors = np.array(velocities)
        
        return velocity_vectors
if __name__ == '__main__':
    #obstacles: represeted by x,y positions
    obstacles = [
        [1,2,3,4]
    ]
    obs =np.load('/home/saleeq/Projects/PySocialForce/pysocialforce/line_endpoints.npy')
    map_endpoint_resolution =0.060 #m/pix

    #obstacle border lines
    obs = (obs -np.mean(obs))*map_endpoint_resolution+[1,1,2,2]
    obstacles = obs
    env = EnvState(obstacles,10)
    min_x, max_x, min_y, max_y, grid_size = -12, 12, -12, 12, 0.25 # Example parameters
    grid_env = GridEnvironment(min_x, max_x, min_y, max_y, grid_size)

    

    

    
    # Set some cells as occupied
    # grid_env.set_occupancy(1.5, 2.5, True)
    # grid_env.set_occupancy(-3.5, -4.5, True)
    
    # Check occupancy
    # print(grid_env.is_occupied(1.5, 2.5))  # True
    # print(grid_env.is_occupied(5, 5))      # False
    
    # # Get cell center
    # print(grid_env.get_cell_center(1, 2))  # Should be within the min_x, max_x, min_y, max_y range
    # print(grid_env.get_cell_center(3, 4))  # Should be within the min_x, max_x, min_y, max_y range
    
    # Print the occupancy grid
    # grid_env.print_grid()
    fig,ax = plt.subplots(figsize=(12,12))
    for obstacle_poses in env.obstacles:
        for p_o in obstacle_poses:
            plt.scatter(p_o[0],p_o[1],color='black',s=1)
            grid_env.set_occupancy(p_o[0],p_o[1],True)
            
    #inflate grid
    grid_env.inflate_obstacles(0.2)
    # Find a path from start to goal
    start = (-7,-5)
    goal = (6,10)
    path = grid_env.a_star(start, goal)
    # print("Path:", path)
    
    
    #plot occuppancy
    for row in range(grid_env.num_rows):
        for col in range(grid_env.num_cols):
            x = grid_env.min_x + col*grid_env.grid_size
            y = grid_env.min_y + row* grid_env.grid_size
            rect_color = 'red' if grid_env.occupancy_grid[row,col] else 'green'
            rect = patches.Rectangle((x,y), grid_env.grid_size,grid_env.grid_size,linewidth=1,edgecolor='black',facecolor=rect_color,alpha=0.2)
            ax.add_patch(rect)
    #plot path
    # Optionally overlay the path
    if path:
        # print(path)
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        ax.scatter(path_x, path_y, c='blue', marker='o', label='Path',s=1)
    ax.set_aspect('equal')
    ax.set_xlim(grid_env.min_x,grid_env.max_x)
    ax.set_ylim(grid_env.min_y,grid_env.max_y)
    # ax.grid()
    print(f"reference path vector {[path]}")
    # cbmpc = CBMPC(obstacles=obstacles,reference_paths=[path],N=10)

    
    # cbmpc.MPC(M=None,a_i=None,C=None)
    plt.show()


    #TODO:reference path in horizon
    