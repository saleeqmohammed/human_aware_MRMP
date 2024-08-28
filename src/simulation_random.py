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
    groups = [[0,2], [5,4]]
    groups=[]
    initial_robot_state = np.array(
        [
            #Warehouse config
            [5.0, 5.0,0.5,0.5,-7,0], #robot 1
            # [10, -5,0.5,0.5,-2.0,5.0],#robot 2
            [-10, 0.0,0.5,0.5,10,0], #robot 3
            # [-10.0, -5.0,0.5,0.5,5,5] #robot 4
            
            #crossover config
            # [-8,0,0.5,0,8,0],
            # [8,0,-0.5,0,-8,0]
        ]
    )
    obs =np.load('/home/saleeq/Projects/PySocialForce/pysocialforce/line_endpoints.npy')
    map_endpoint_resolution =0.060 #m/pix



    #obstacle border lines
    # obs = (obs -np.mean(obs))*map_endpoint_resolution+[1,1,2,2]
    obs = []
    def generate_random_env(n_items):
        items=[]
        for _ in range(n_items):
            width =random.randint(1,10)
            height = random.randint(1,10)
            x = random.randint(math.ceil(-12+width/2) ,math.floor(12-width/2))
            y = random.randint(math.ceil(-12+height/2),math.floor(12-height/2))
            box = rectangle_with_diagonals(x,y=y,width=width,length=height)
            items.append(box)
        return items


    #Define walls
    walls = np.array([
        [-12,12,12,12],
        [12,12,-12,12],
        [-12,12,-12,-12],
        [-12,-12,-12,12],
      
    ])  
    
    #create an inventry of items
    box1 = rectangle_with_diagonals(x=0,y=0,width=2,length=2)
    box2 = rectangle_with_diagonals(x=-5, y=8, width=8, length=2)
    box3 = rectangle_with_diagonals(x=5,y=-5,width=6,length=4)
    box4 = rectangle_with_diagonals(x=-7,y=8,width=2,length=2)
    box5 = rectangle_with_diagonals(x=6,y=-8,width=1,length=4,)
    box6 = rectangle_with_diagonals(x=4,y=-8,width=1,length=4,)
    pent1 = generate_polygon((5,5),[2,4,5,3])
    # hex1 = generate_polygon()
    #put the items together to generate a problem                        
    # items = generate_random_env(n_items=4)
    # items=[box1,pent1,box4,box2,box5,box6]#,box2,box3,box4,pent1]
    
    #Items for scenario 1
    items_scn_1=[
    rectangle_with_diagonals(x=0,y=9,width=3,length=1),
    rectangle_with_diagonals(x=0,y=7,width=3,length=1),
    rectangle_with_diagonals(x=3,y=3,width=1,length=1),
    rectangle_with_diagonals(x=3,y=-3,width=1,length=1),
    rectangle_with_diagonals(x=-7,y=1,width=2,length=2),
    rectangle_with_diagonals(x=-7,y=-2,width=2,length=2),
    rectangle_with_diagonals(x=5,y=-9,width=3,length=2)
    
    ]
    #Initial robot_state scenario 1
    initial_robot_state_scn_1=np.array(
        [
            #x y vx vy gx gy 
            [-7,8,0.5,-0.5,0,-9],
            # [7,8,0,-0.5,5,-6],
            [-7,-9,0.5,0.5,1,3]
        ]
    )
    initial_ped_state_scn_1=np.array(
        [
            [-5,1,0.5,0,3,1],
            [3,-1,-0.5,0,-5,-1],
            [6,-7,0,0.5,4,3]
        ]
    )

    #Items for scenario2 
    items_scn_2=[
        rectangle_with_diagonals(7,7,2,2),
        rectangle_with_diagonals(3,0,1,1),
        rectangle_with_diagonals(-3,0,1,1),
    
        generate_polygon((-7,-7),[2,2,3,3,3]),
        rectangle_with_diagonals(x=7,y=-9,width=5,length=2),
        rectangle_with_diagonals(x=-5,y=9,width=6,length=1),
        
    ]
    initial_robot_state_scn_2=np.array(
        [
            [-2,6,0.5,-0.5,3,-9],
            [-3,-9,0.5,0,7,5]
        ]
    )
    initial_ped_state_scn_2=np.array(
        [
            [-3,-1,-0.5,-0.5,3,-6],
            [4,-1,-0.5,-0.5,-3,-8],
            
        ]
    )
    # groups=[[0,1]]

    #items for scenario 3
    items_scn_3=[
        rectangle_with_diagonals(5,0,2.5,4),
        rectangle_with_diagonals(-5,0,2,4),
        rectangle_with_diagonals(-9,5,1,5),
        rectangle_with_diagonals(0,10,6,1),
        rectangle_with_diagonals(10,10,1,1),
        rectangle_with_diagonals(-7,-7,3,2),
        generate_polygon((8,-9),[2,2,2,1,2,2]),
        [[-6,-4,0,0],[-4,4,5,5],[-4,4,5.1,5.1]]


    ]
    initial_robot_state_scn_3=np.array(
        [
            [2,6,0.5,-0.5,3,-9],
            [-3,-9,0.5,0,7,5],
            [-7.4,-3,0.5,0,1,7]
        ]
    )
    initial_ped_state_scn_3=np.array(
        [
            [-3,-1,-0.5,-0.5,5,10],
            [4,-3,-0.5,-0.5,-3,-3],
            
            
        ]
    )
    initial_robot_state = initial_robot_state_scn_1
    initial_ped_state = initial_ped_state_scn_1
    items = items_scn_1
    #put all the walls
    for wall in walls:
        print(f"wall {wall}")
        obs.append(wall)

    for item in items:
        for line in item:
            print(f"Obstacle line added: {line}")
            obs.append(line)

    obs = np.array(obs)

    '''
    Check feasibility of the arrangement
    '''
    env = EnvState(obstacles=obs,resolution=10)
    min_x, max_x, min_y, max_y, grid_size = -15, 15, -15, 15, 0.25 
    grid_env = GridEnvironment(min_x,max_x,min_y,max_y,grid_size)
    #set occuppancy for obstacles
    for obstacle_poses in env.obstacles:
        for p_o in obstacle_poses:
            grid_env.set_occupancy(p_o[0],p_o[1],True)

    #inflate obstacles
    grid_env.inflate_obstacles(0.6)
    occuppancy = grid_env.occupancy_grid
    occuppancy_fraction = np.sum(occuppancy)/occuppancy.size
    print(f"grid occupancy: {100*occuppancy_fraction}%")
        #     reference_paths =[]
        #     #plan paths
        #     for i in range(len(initial_robot_state)):
        #         robot = initial_robot_state[i]
        #         start = (robot[0],robot[1])
        #         goal = (robot[4],robot[5])
        #         path = grid_env.a_star(start,goal)
        #         reference_paths.append(path)
        #     return grid_env,obs,reference_paths
        # grid_env,obs,reference_paths =generate_problem()
  
        # for path in reference_paths:
        #     if len(path) <1:
        #         generate_problem()

        # n_itr = 1000
        # while conflict_found:
        #     conflict_found = False
        #     n_itr+=1
        #     #replan paths
        #     for i in range(len(initial_robot_state)):
        #         path = reference_paths[i]
        #         #check conflict with previous paths  
        #         for j in range(i):
        #             other_path = reference_paths[j]
        #             for step in range(min(len(path),len(other_path))):
        #                 if np.linalg.norm(np.array(path[step])-np.array(other_path[step])) < 0.7:
        #                     conflict_found = True
        #                     conflict_time = step
        #                     conflict_pos = path[step]

        #                     #block the conflicting cell at conflicting time
        #                     occuppancy = grid_env.occupancy_grid

        #                     x,y= conflict_pos
        #                     grid_env.block_range(x,y,1.0)
        #                     path = grid_env.a_star(start,goal)
        #                     reference_paths[i] = path

        #                     grid_env.occupancy_grid = occuppancy
        #                     # grid_env.inflate_obstacles(0.2)

        #                     break
        #                 else:
        #                     conflict_found = False
        #             if conflict_found:
        #                 break

   
    
    # initiate the simulator,
    s = hs.Simulator(
        initial_ped_state,
        initial_robot_state,
        groups=groups,
        obstacles=obs,
        config_file=Path(__file__).resolve().parent.joinpath("simulation.toml"),
    )
    # update 80 steps
    s.step(50)
    human_colors = len(initial_ped_state)*['red']
    with hs.plot.SceneVisualizer(scene=s,output= "/home/saleeq/catkin_ws/src/human_aware_MRMP/images/output_demo",agent_colors=human_colors) as sv:
        sv.animate()
        # sv.plot()
