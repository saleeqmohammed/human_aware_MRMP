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
    
    def _get_cell_indices(self, x, y):
        """Convert (x, y) coordinates to grid cell indices."""
        col = int((x - self.min_x) // self.grid_size)
        row = int((y - self.min_y) // self.grid_size)
        return (row, col)
    
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
    def __init__(self):
        self.cost = None
        self.solution = None
        self.constraints=None
    def conflicts(self):
        return 0

class O_list:
    def __init__(self):
        self.lowest =Node
    def get_lowest(self):
        return self.lowest
    def insert(R:Node):
        pass
class CBMPC:
    def __init__(self,obstacles,N):
        self.obstacles = obstacles
        self.N = N 
        self.agents=[]   

        # Define weights
        self.Q = np.diag([12.0, 12.0, 1.0])  # Weight for state tracking error
        self.R = np.diag([12.0, 0.05])       # Weight for control effort
        self.P_term = np.diag([12.5,12.5,10.0])  # Weight for goal tracking error
        #constrint parameters
        self.epsilon_g = 0.2 #goal tolerance
        self.D = 1.04 #robot footprint
        self.epsilon_r = 0.05 #robot robot collision tolerance   
        self.kr = 10e6 #inter robot collision tolerance slcak coefficient
        self.ko = 10e6 #obstacle collision tolerance slack coefficient
        #mpc parameters
        self.T = 0.2 #mpc time step
        self.nx = 3 #number of states [x,y,theta]
        self.nu = 2 #number of controls [v, omega]

        #

    def MPC_problem(self,pos,goal) -> Tuple:
        """
        tuple M(Obstacle set, Initial robot positions, Final robot positions, Horizon length) : Problem definition
        """
        M = (self.obstacles,pos,goal,self.N)
        return M
    @staticmethod
    def trajectory_length(solution):
        X_i = solution.poses
    @staticmethod
    def euclidian_distance(pos,goal):
        return np.sqrt(np.transpose(pos - goal)*(pos-goal))
    
    def constraint_1(self):
        """
        Constraint to look for convergence at time instant k
        Lim k->inf ||x_i^d - x_i^k|| <= epsilon_g
        """

        self.agents
    def MPC(self,M,a_i,C):
        """
        Model Predictive control subroutine
        MPC(M,a_i,C)
        Inputs :-
        M : Problem definition
        a_i : Agent index
        C = A.constraints : set of agent specific constraints of the form (a_i, a_j,[t_c1,t_c2]) for all a_i != a_j
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
        #Define symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(v, omega)
        delta_r = ca.SX.sym('delta_r')
        delta_o = ca.SX.sym('delta_o')
        # System dynamics
        rhs = ca.vertcat(v*ca.cos(theta), v*ca.sin(theta), omega)

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
        st = X[:, 0]
        g.append(st - ref_trajectory[:, 0])
        # Formulate the optimization problem with weights
        for k in range(N):
            st = X[:, k]
            con = U[:, k]
            # Cost function: weighted state deviation from trajectory + weighted control effort + terminal cost
            obj += ca.mtimes([(st - ref_trajectory[:, k]).T, Q, (st - ref_trajectory[:, k])]) + ca.mtimes([con.T, R, con]) + ca.mtimes([(X[:,N]-goal_state).T,P_term,(X[:,N]-goal_state)]) #+ kr*delta_r + ko*delta_o
            st_next = X[:, k+1]
            f_value = f(st, con)
            st_next_euler = st + T*f_value
            g.append(st_next - st_next_euler)
        # Define optimization variables
        OPT_variables = ca.vertcat(ca.reshape(X, nx*(N+1), 1), ca.reshape(U, nu*N, 1))
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

        # Define the bounds
        lbx = []
        ubx = []

        # State bounds
        for _ in range(N+1):
            lbx.extend([-ca.inf, -ca.inf, -ca.inf])
            ubx.extend([ca.inf, ca.inf, ca.inf])

        # Control bounds
        for _ in range(N):
            lbx.extend([-1.0, -np.pi/4])  # v lower bound, omega lower bound
            ubx.extend([1.0, np.pi/4])   # v upper bound, omega upper bound

        lbx = ca.vertcat(*lbx)
        ubx = ca.vertcat(*ubx)

        # Constraint bounds (equality constraints)
        lbg = ca.vertcat(*([0] * nx * (N + 1)))
        ubg = lbg
        # Initial state and trajectory reference states
        x0 = np.array([0.0, 0.0, 0.0])
        # Define a trajectory as an example
        # For example, a linear trajectory from (0,0) to (5,5)
        trajectory = np.linspace([0.0, 0.0, 0.0], [5.0, 5.0, 0.0], N+1)
        goal = np.array([5.0, 5.0, 0.0])
        # Flatten the trajectory into a single vector
        p = np.concatenate((trajectory.flatten(), goal))

        # Initial guess
        x_init = np.zeros((nx*(N+1) + nu*N, 1))

        # Solve the problem
        sol = solver(x0=x_init, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

        # Extract solution
        sol_x = sol['x'].full().flatten()
        x_solution = sol_x[:nx*(N+1)].reshape((N+1, nx))
        u_solution = sol_x[nx*(N+1):].reshape((N, nu))

        print("Optimal states trajectory: \n", x_solution)
        print("Optimal control inputs: \n", u_solution)

        # Convert ref_trajectory to numerical format
        ref_trajectory_numerical = trajectory.T  # Transpose to match shape (nx, N+1)

        # Plotting and animation
        def animate(i):
            plt.clf()
            plt.plot(ref_trajectory_numerical[0, :], ref_trajectory_numerical[1, :], 'g--', label='Reference Trajectory')
            plt.plot(x_solution[:, 0], x_solution[:, 1], 'b-', label='Agent Path')
            plt.scatter(x_solution[i, 0], x_solution[i, 1], color='red')  # Current position of the agent
            plt.xlim(min(ref_trajectory_numerical[0, :])-1, max(ref_trajectory_numerical[0, :])+1)
            plt.ylim(min(ref_trajectory_numerical[1, :])-1, max(ref_trajectory_numerical[1, :])+1)
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('Agent Path and Reference Trajectory')
            plt.legend()

        # Plot setup
        fig = plt.figure()
        ani = FuncAnimation(fig, animate, frames=N+1, interval=200, repeat=True)

        ani.save('animation.gif',writer='pillow',fps=5)
        plt.show()


    def SIC(self,solution):
        """
        Method to calculate sum of induvidual costs
        J_n = J_c + J_f

        Input: set of x_i in solution
        """
        #trajectory length till now
        J_c = self.trajectory_length(solution)
        #distance left
        J_f = self.euclidaian_distance()
        pass
    def Solution(self,M):
        R = Node()
        for a_i in self.agents:
            R.solution[a_i] = self.MPC(M,a_i,{})
        R.cost =self.SIC(R.solution)
        O= O_list()
        O.insert(R)
        while(True):
            P = O.get_lowest()
            if len(P.conflicts()) ==0:
                return P.solution
            C = P.conflicts()[1]
            for a_i in C.agents:
                A = Node()
                A.constraints = P.constraints + ()
                A.solution[a_i] = self.MPC(M,a_i,A.constraints)
                A.cost = self.SIC(A.solution)
                O.insert(A)



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
    min_x, max_x, min_y, max_y, grid_size = -12, 12, -12, 12, 1 # Example parameters
    grid_env = GridEnvironment(min_x, max_x, min_y, max_y, grid_size)

    

    

    
    # Set some cells as occupied
    # grid_env.set_occupancy(1.5, 2.5, True)
    # grid_env.set_occupancy(-3.5, -4.5, True)
    
    # Check occupancy
    print(grid_env.is_occupied(1.5, 2.5))  # True
    print(grid_env.is_occupied(5, 5))      # False
    
    # Get cell center
    print(grid_env.get_cell_center(1, 2))  # Should be within the min_x, max_x, min_y, max_y range
    print(grid_env.get_cell_center(3, 4))  # Should be within the min_x, max_x, min_y, max_y range
    
    # Print the occupancy grid
    # grid_env.print_grid()
    fig,ax = plt.subplots(figsize=(12,12))
    for obstacle_poses in env.obstacles:
        for p_o in obstacle_poses:
            plt.scatter(p_o[0],p_o[1],color='black',s=1)
            grid_env.set_occupancy(p_o[0],p_o[1],True)
            
    # Find a path from start to goal
    start = (-7,-5)
    goal = (6,10)
    path = grid_env.a_star(start, goal)
    print("Path:", path)
    
    
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
        print(path)
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        ax.scatter(path_x, path_y, c='blue', marker='o', label='Path')
    ax.set_aspect('equal')
    ax.set_xlim(grid_env.min_x,grid_env.max_x)
    ax.set_ylim(grid_env.min_y,grid_env.max_y)
    # ax.grid()
    cbmpc = CBMPC(obstacles=obstacles,N=10)
    # mpc.MPC_problem()
    cbmpc.MPC(M=None,a_i=None,C=None)
    plt.show()


    #TODO:reference path in horizon
    