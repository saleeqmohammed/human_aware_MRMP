from pathlib import Path
import numpy as np
import pysocialforce as psf


if __name__ == "__main__":
    # initial states, each entry is the position, velocity and goal of a pedestrian in the form of (px, py, vx, vy, gx, gy)
    initial_state = np.array(
        [
            [2.0, -6.0, -0.5, -0.5, -3.0, 7.0],
            [-5.0, 4.0, -0.5, -0.5, 5, -5.0],
            [6.0, 1.0, 0.0, 0.5, -5.0, 2.0],
            # [1.0, 0.0, 0.0, 0.5, 2.0, 10.0],
            # [2.0, 0.0, 0.0, 0.5, 3.0, 10.0],
            # [3.0, 0.0, 0.0, 0.5, 4.0, 10.0],
        ]
    )
    # social groups informoation is represented as lists of indices of the state array
    groups = [[1],[0], [2]]
    # list of linear obstacles given in the form of (x_min, x_max, y_min, y_max)
    obs = [[-1, -1, -1, 11], [3, 3, -1, 11]]
    obs =[
        [-11,12,12,12],#B1
        [12,12,-12,12]#B2
        #B3

    ]
    obs =np.load('/home/saleeq/Projects/PySocialForce/pysocialforce/line_endpoints.npy')
    map_endpoint_resolution =0.0550 #m/pix
    obs = (obs -np.mean(obs))*map_endpoint_resolution+[1,1,2,2]
    # obs = [[1, 2, 7, 8]] 
    ref_map = np.load('/home/saleeq/Projects/PySocialForce/pysocialforce/binary_map.npy')
    coords = np.where(ref_map==1)
    print(np.max(coords)*0.050-12)
    # obs = None
    # initiate the simulator,
    s = psf.Simulator(
        initial_state,
        groups=groups,
        obstacles=obs,
        config_file=Path(__file__).resolve().parent.joinpath("example.toml"),
    )
    # update 80 steps
    s.step(50)

    with psf.plot.SceneVisualizer(s, "images/example") as sv:
        sv.animate()
        # sv.plot()
