from pathlib import Path
import numpy as np
import hacbs as hs
import random

if __name__ == "__main__":
    # initial states, each entry is the position, velocity and goal of a pedestrian in the form of (px, py, vx, vy, gx, gy)
    initial_ped_state = np.array(
        [
            [2.0, -6.0, 0.5, -0.5, -3.0, 7.0], #blue
            [-4.5, 4.0, -0.5, 0.0, 5, -5.0], #cyan
            [6.0, 1.0, 0.0, 0.5, -5.0, 2.0], #yellow
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
            [10, -5,0.5,0.5,-2.0,5.0],#robot 2
            [-10, 0.0,0.5,0.5,10,0], #robot 3
            [-10.0, -5.0,0.5,0.5,5,5] #robot 4
            
            #crossover config
            # [-8,0,0.5,0,8,0],
            # [8,0,-0.5,0,-8,8]
        ]
    )
    obs =np.load('/home/saleeq/Projects/PySocialForce/pysocialforce/line_endpoints.npy')
    map_endpoint_resolution =0.060 #m/pix



    #obstacle border lines
    obs = (obs -np.mean(obs))*map_endpoint_resolution+[1,1,2,2]
    # obs = np.array([
    #     [-12,12,12,12],
    #     [12,12,-12,12],
    #     [-12,12,-12,-12],
    #     [-12,-12,-12,12]
    # ])

 

    def generate_random_shapes(x_min, x_max, y_min, y_max, num_shapes):
        output = np.zeros((num_shapes + 4, 4))

        # Define the border lines
        borders = [
            [x_min, x_max, y_max, y_max],  # Top border
            [x_max, x_max, y_min, y_max],  # Right border
            [x_min, x_max, y_min, y_min],  # Bottom border
            [x_min, x_min, y_min, y_max]   # Left border
        ]

        # Insert the borders into the output array
        for i in range(4):
            output[i] = borders[i]

        for i in range(num_shapes):
            shape_type = random.choice(['rectangle', 'triangle', 'perpendicular'])
            
            if shape_type == 'rectangle':
                x1, x2 = sorted([random.uniform(x_min, x_max) for _ in range(2)])
                y1, y2 = sorted([random.uniform(y_min, y_max) for _ in range(2)])
                output[i + 4] = [x1, x2, y1, y2]

            elif shape_type == 'triangle':
                x1, x2, x3 = random.uniform(x_min, x_max), random.uniform(x_min, x_max), random.uniform(x_min, x_max)
                y1, y2, y3 = random.uniform(y_min, y_max), random.uniform(y_min, y_max), random.uniform(y_min, y_max)
                xmin, xmax = min(x1, x2, x3), max(x1, x2, x3)
                ymin, ymax = min(y1, y2, y3), max(y1, y2, y3)
                output[i + 4] = [xmin, xmax, ymin, ymax]

            elif shape_type == 'perpendicular':
                # Start point
                x1, y1 = random.uniform(x_min, x_max), random.uniform(y_min, y_max)
                # Small perpendicular offset
                dx, dy = random.choice([(1, 0), (0, 1)])  # Either horizontal or vertical
                length = random.uniform(0.1, 2.0)  # Small length for perpendicular line
                x2, y2 = x1 + dx * length, y1 + dy * length
                output[i + 4] = [min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)]

        return output

    # Example usage
    x_min, x_max = -12, 12
    y_min, y_max = -12, 12
    num_obstacles = 5  # Number of random lines to generate

    # obs = generate_random_shapes(x_min, x_max, y_min, y_max, num_obstacles)


    # initiate the simulator,
    s = hs.Simulator(
        initial_ped_state,
        initial_robot_state,
        groups=groups,
        obstacles=obs,
        config_file=Path(__file__).resolve().parent.joinpath("simulation.toml"),
    )
    # update 80 steps
    s.step(20)

    with hs.plot.SceneVisualizer(s, "/home/saleeq/catkin_ws/src/human_aware_MRMP/images/output2") as sv:
        sv.animate()
        # sv.plot()
