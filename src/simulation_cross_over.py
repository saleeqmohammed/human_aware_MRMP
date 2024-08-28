from pathlib import Path
import numpy as np
import hacbs as hs
from hacbs.planning import GridEnvironment
from hacbs.scene import EnvState
import random
import math

def rectangle_with_diagonals(x, y, width, length):
    # Calculate the half-width and half-length
    half_width = width / 2
    half_length = length / 2
    
    # Calculate the four corners of the rectangle
    top_left = (x - half_width, y + half_length)
    top_right = (x + half_width, y + half_length)
    bottom_left = (x - half_width, y - half_length)
    bottom_right = (x + half_width, y - half_length)
    
    # Calculate the lines of the rectangle
    line1 = [top_left[0], top_right[0], top_left[1], top_right[1]]       # Top line
    line2 = [top_right[0], bottom_right[0], top_right[1], bottom_right[1]] # Right line
    line3 = [bottom_right[0], bottom_left[0], bottom_right[1], bottom_left[1]] # Bottom line
    line4 = [bottom_left[0], top_left[0], bottom_left[1], top_left[1]]     # Left line
    
    # Calculate the diagonal lines of the rectangle
    diagonal1 = [top_left[0], bottom_right[0], top_left[1], bottom_right[1]]  # Diagonal from top-left to bottom-right
    diagonal2 = [top_right[0], bottom_left[0], top_right[1], bottom_left[1]]  # Diagonal from top-right to bottom-left
    
    # Return the lines including the diagonals
    return [line1, line2, line3, line4]#, diagonal1, diagonal2]
def generate_polygon(centroid, side_lengths):
    num_sides = len(side_lengths)
    
    if num_sides < 3:
        raise ValueError("A polygon must have at least 3 sides")
    
    # Centroid coordinates
    cx, cy = centroid
    
    # Calculate the angle between each vertex (in radians)
    angles = np.linspace(0, 2 * np.pi, num_sides + 1)[:-1]
    
    # Initialize the list of vertices
    vertices = []
    
    # Initial angle (can start from any angle, here we start from 0)
    current_angle = 0
    
    # Calculate each vertex based on the centroid, angle, and side length
    for i in range(num_sides):
        length = side_lengths[i % len(side_lengths)]  # If side_lengths has fewer elements, it will repeat
        # Compute the position of the vertex
        x = cx + length * np.cos(current_angle)
        y = cy + length * np.sin(current_angle)
        vertices.append((x, y))
        
        # Update the angle for the next vertex
        if i < num_sides - 1:
            next_length = side_lengths[(i + 1) % len(side_lengths)]
            # Compute the angle for the next vertex using the cosine rule
            angle_increment = np.arccos((length**2 + next_length**2 - length**2) / (2 * length * next_length))
            current_angle += angle_increment
    
    # Form the polygon by connecting vertices
    polygon_edges = []
    for i in range(num_sides):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % num_sides]
        polygon_edges.append([x1, x2, y1, y2])
    
    return polygon_edges


def is_overlapping(rect1, rect2):
    # Check if two rectangles overlap
    xmin1, xmax1, ymin1, ymax1 = rect1
    xmin2, xmax2, ymin2, ymax2 = rect2
    return not (xmax1 <= xmin2 or xmax2 <= xmin1 or ymax1 <= ymin2 or ymax2 <= ymin1)

def generate_obstacles(num_obstacles, x_range=(-10, 10), y_range=(-10, 10), size_range=(1, 5)):
    obstacles = []
    max_attempts = 1000  # Limit to avoid infinite loop

    for _ in range(num_obstacles):
        attempts = 0
        placed = False
        while not placed and attempts < max_attempts:
            width = np.random.uniform(size_range[0], size_range[1])
            height = np.random.uniform(size_range[0], size_range[1])
            xmin = np.random.uniform(x_range[0], x_range[1] - width)
            xmax = xmin + width
            ymin = np.random.uniform(y_range[0], y_range[1] - height)
            ymax = ymin + height

            new_obstacle = [xmin, xmax, ymin, ymax]
            if all(not is_overlapping(new_obstacle, obs) for obs in obstacles):
                obstacles.append(new_obstacle)
                placed = True
            attempts += 1
        
        if attempts >= max_attempts:
            print("Warning: Unable to place obstacle after many attempts.")

    return obstacles
def get_obstacle_lines(obstacles):
    lines = []
    for obstacle in obstacles:
        xmin, xmax, ymin, ymax = obstacle
        # Horizontal lines
        lines.append([xmin, xmax, ymin, ymin])  # Bottom edge
        lines.append([xmin, xmax, ymax, ymax])  # Top edge
        # Vertical lines
        lines.append([xmin, xmin, ymin, ymax])  # Left edge
        lines.append([xmax, xmax, ymin, ymax])  # Right edge
    return lines
def get_checkerboard_lines(obstacles, interval=0.5):
    lines = []
    for obstacle in obstacles:
        xmin, xmax, ymin, ymax = obstacle
        # Generate vertical lines
        x = xmin
        while x <= xmax:
            lines.append([x, x, ymin, ymax])
            x += interval
        # Generate horizontal lines
        y = ymin
        while y <= ymax:
            lines.append([xmin, xmax, y, y])
            y += interval
    return lines

def is_point_within_obstacles(point, obstacles):
    x, y = point
    for obs in obstacles:
        xmin, xmax, ymin, ymax = obs
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return True
    return False

def generate_agents(num_agents, x_range=(-10, 10), y_range=(-10, 10), obstacles=[]):
    agents = []
    max_attempts = 1000

    for _ in range(num_agents):
        attempts = 0
        placed = False
        while not placed and attempts < max_attempts:
            # Generate start point
            xs = np.random.uniform(x_range[0], x_range[1])
            ys = np.random.uniform(y_range[0], y_range[1])
            
            # Generate goal point
            xg = np.random.uniform(x_range[0], x_range[1])
            yg = np.random.uniform(y_range[0], y_range[1])
            
            if is_point_within_obstacles((xs, ys), obstacles) or is_point_within_obstacles((xg, yg), obstacles):
                attempts += 1
                continue
            
            # Determine velocities
            vx = 0.5 if xg >= xs else -0.5
            vy = 0.5 if yg >= ys else -0.5

            new_agent = [xs, ys, vx, vy, xg, yg]
            
            # Check if the new agent overlaps with any existing agents
            if all(not (agent[0] == xs and agent[1] == ys and agent[4] == xg and agent[5] == yg) for agent in agents):
                agents.append(new_agent)
                placed = True
            attempts += 1
        
        if attempts >= max_attempts:
            print("Warning: Unable to place agent after many attempts.")
    
    return np.array(agents)

def calculate_total_area(obstacles):
    total_area = 0
    for obstacle in obstacles:
        xmin, xmax, ymin, ymax = obstacle
        width = xmax - xmin
        height = ymax - ymin
        total_area += width * height
    return total_area

if __name__ == "__main__":
    # initial states, each entry is the position, velocity and goal of a pedestrian in the form of (px, py, vx, vy, gx, gy)
    initial_ped_state = np.array(
        [
            [2.0, -6.0, 0.5, -0.5, -3.0, 7.0], #blue
            [-4.5, 4.0, -0.5, 0.0, 5, -5.0], #cyan
            # [6.0, 1.0, 0.0, 0.5, -5.0, 2.0], #yellow
            # [1.0, 0.0, 0.0, 0.5, 2.0, 10.0],
            # [2.0, 0.0, 0.0, 0.5, 3.0, 10.0],
            # [3.0, 0.0, 0.0, 0.5, 4.0, 10.0],
        ]
    )
    # social groups informoation is represented as lists of indices of the state array
    # groups = [[0,2], [5,4]]
    groups=[]
    initial_robot_state = np.array(
        [

            
            #crossover config
            [-8,0,0.5,0,8,0],#left
            [8,0,-0.5,0,-8,0],#right
            [0,8.0,0,-0.5,0,-8],#top
            # [0,-8,0,0.5,0,8]#bottom
        ]
    )
 


 
    
   
    obstacles=[]
    obs = get_obstacle_lines(obstacles)
    n_robots =2
    n_humans= 2
    
    s = hs.Simulator(
        initial_ped_state,
        initial_robot_state,
        groups=groups,
        obstacles=obs,
        config_file=Path(__file__).resolve().parent.joinpath("simulation.toml"),
    )
    # update 80 steps
    s.step(30)
    human_colors = len(initial_ped_state)*['red']
    with hs.plot.SceneVisualizer(scene=s,output= "/home/saleeq/catkin_ws/src/human_aware_MRMP/images/output_demo",agent_colors=human_colors) as sv:
        sv.animate()
        # sv.plot()
