#!/usr/bin/env python3
from typing import Tuple
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from pysocialforce.scene import EnvState
import matplotlib.pyplot as plt
from matplotlib import patches
import heapq


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
        f_score = {start_indices: self.heuristic(start_indices, goal_indices)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal_indices:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.grid_size
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_indices)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # No path found

    def get_neighbors(self, indices):
        """Get neighboring cells (up, down, left, right) of the current cell."""
        row, col = indices
        neighbors = []
        
        if row > 0:  # Up
            neighbors.append((row - 1, col))
        if row < self.num_rows - 1:  # Down
            neighbors.append((row + 1, col))
        if col > 0:  # Left
            neighbors.append((row, col - 1))
        if col < self.num_cols - 1:  # Right
            neighbors.append((row, col + 1))
        
        # Filter out occupied cells
        return [neighbor for neighbor in neighbors if not self.occupancy_grid[neighbor]]

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        # Convert path from grid indices to actual coordinates
        return [self.get_cell_center(row, col) for row, col in path]




class GlobalPlanner:
    """
    Djikstra planner to generate reference path
    """
    def __init__(self, obstacles):
        self.map = None
        self.step_resolution =10
    def make_plan(self,pos,goal):
        samples = int(np.linalg.norm((pos[0]-goal[0],pos[1]-goal[1])*self.step_resolution))
        path = np.array(
            list(
                zip(np.linspace(pos[0],goal[0],samples),np.linspace(pos[1],goal[1],samples))
            )
        )
        return path
    
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
        self.Q = np.diag([12.0, 12.0])   
        self.agents=[]      
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

        pass
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

    

    mpc = CBMPC(obstacles=obstacles,N=10)
    reference_path =GlobalPlanner(env.obstacles)
    # reference_path.make_plan()
    # mpc.MPC_problem()
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
    start = (0,0)
    goal = (3,-3)
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
        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        ax.scatter(path_x, path_y, c='blue', marker='o', label='Path')
    ax.set_aspect('equal')
    ax.set_xlim(grid_env.min_x,grid_env.max_x)
    ax.set_ylim(grid_env.min_y,grid_env.max_y)
    # ax.grid()
    plt.show()