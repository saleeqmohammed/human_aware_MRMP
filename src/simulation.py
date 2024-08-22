from pathlib import Path
import numpy as np
import hacbs as hs


if __name__ == "__main__":
    # initial states, each entry is the position, velocity and goal of a pedestrian in the form of (px, py, vx, vy, gx, gy)
    initial_ped_state = np.array(
        [
            # [2.0, -6.0, 0.5, -0.5, -3.0, 7.0], #blue
            [-4.5, 4.0, -0.5, 0.0, 5, -5.0], #cyan
            [6.0, 1.0, 0.0, 0.5, -5.0, 2.0], #yellow
            # [1.0, 0.0, 0.0, 0.5, 2.0, 10.0],
            # [2.0, 0.0, 0.0, 0.5, 3.0, 10.0],
            [3.0, 0.0, 0.0, 0.5, 4.0, 10.0],
        ]
    )
    # social groups informoation is represented as lists of indices of the state array
    groups = [[0,2]]
    initial_robot_state = np.array(
        [
            [5.0, 5.0,0.5,0.5,-10,0], #robot 1
            [10, -5,0.5,0.5,-4.0,4.0],#robot 2
            # [-10, 0.0,0.5,0.5,10,0], #robot 3
            [-10.0, -5.0,0.5,0.5,5,5] #robot 5
        ]
    )
    obs =np.load('/home/saleeq/Projects/PySocialForce/pysocialforce/line_endpoints.npy')
    map_endpoint_resolution =0.060 #m/pix

    #obstacle border lines
    obs = (obs -np.mean(obs))*map_endpoint_resolution+[1,1,2,2]
    # initiate the simulator,
    s = hs.Simulator(
        initial_ped_state,
        initial_robot_state,
        groups=groups,
        obstacles=obs,
        config_file=Path(__file__).resolve().parent.joinpath("simulation.toml"),
    )
    # update 80 steps
    s.step(60)

    with hs.plot.SceneVisualizer(s, "/home/saleeq/catkin_ws/src/human_aware_MRMP/images/example") as sv:
        sv.animate()
        # sv.plot()
