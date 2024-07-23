#!/usr/bin/env python3
from typing import Tuple
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

class GlobalPlanner:
    """
    Djikstra planner to generate reference path
    """
    def __init__(self, obstacles):
        self.map = None
        pass

class CBMPC:
    def __init__(self,obstacles,N):
        self.obstacles = obstacles
        self.N = N
        
    def MPC_problem(self,pos,goal) -> Tuple:
        """
        tuple M(Obstacle set, Initial robot positions, Final robot positions, Horizon length) : Problem definition
        """
        M = (self.obstacles,pos,goal,self.N)
        return M
    
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

if __name__ == '__main__':
    obs =np.load('/home/saleeq/Projects/PySocialForce/pysocialforce/line_endpoints.npy')
    map_endpoint_resolution =0.060 #m/pix

    #obstacle border lines
    obstacles = (obs -np.mean(obs))*map_endpoint_resolution+[1,1,2,2]
    mpc = CBMPC(obstacles=obstacles,N=10)
    mpc.MPC_problem()