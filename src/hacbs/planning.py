#!/usr/bin/env python3
from typing import Tuple


class CBMPC:
    def __init__(self,obstacles,N):
        self.obstacles = obstacles
        self.N = N
        
    """
    tuple M(Obstacle set, Initial robot positions, Final robot positions, Horizon length) : Problem definition
    """
    def MPC_problem(self,pos,goal) -> Tuple:
        M = (self.obstacles,pos,goal,self.N)
        return M
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

    def MPC(self,M,a_i,C):
        pass

if __name__ == '__main__':
    pass