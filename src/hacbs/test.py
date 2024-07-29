#!/usr/bin/env python3
import casadi as ca

# Define symbolic variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')

# Define objective function
objective = x**2 + y**2

# Define constraints
constraint = x + y - 1

# Setup the optimization problem

# Define optimization variables
opt_vars = ca.vertcat(x, y)

# Define problem
nlp = {
    'f': objective,    # Objective function
    'x': opt_vars,     # Optimization variables
    'g': constraint    # Constraints
}

# Create and configure solver
opts = {'ipopt': {'print_level': 0}}  # Options for IPOPT solver
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

# Define bounds for variables and constraints
lbx = [-ca.inf, -ca.inf]  # Lower bounds for x and y
ubx = [ca.inf, ca.inf]    # Upper bounds for x and y
lbg = [0]                 # Lower bound for the constraint (equality)
ubg = [0]                 # Upper bound for the constraint (equality)

# Solve the problem
x0 = [0, 0]  # Initial guess
sol = solver(x0=x0, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)  # Solve
solution = sol['x'].full()

print(f"Optimal solution: {solution}")
