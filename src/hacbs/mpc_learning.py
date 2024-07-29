#!/usr/bin/env python3
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
 
#define parameters
N =20 #prediction horizon
dt =0.1 # time tep
lambda_v = 0.1 #weight control effor for omega
lambda_omega =0.1
 #define symbolic variables
 #state
x= ca.MX.sym('x')
y =ca.MX.sym('y')
theta = ca.MX.sym('theta')
#control
v = ca.MX.sym('v')
omega = ca.MX.sym('omega')