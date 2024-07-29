import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define parameters
T = 0.2  # Time step
N = 60   # Prediction horizon

# Define state and control dimensions
nx = 3  # Number of states [x, y, theta]
nu = 2  # Number of control inputs [v, omega]

# Define symbolic variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(x, y, theta)
v = ca.SX.sym('v')
omega = ca.SX.sym('omega')
controls = ca.vertcat(v, omega)

# System dynamics
rhs = ca.vertcat(v*ca.cos(theta), v*ca.sin(theta), omega)

# Define the function f(x,u)
f = ca.Function('f', [states, controls], [rhs])

# Define optimization variables
U = ca.SX.sym('U', nu, N)  # Control inputs for N steps
P = ca.SX.sym('P', nx * (N + 2))  # Reference states for N+1 steps + goal state
X = ca.SX.sym('X', nx, N+1)  # States for N+1 steps

# Objective function and constraints
obj = 0  # Objective function
g = []   # Constraints

# Define trajectory reference states
ref_trajectory = P[:nx * (N+1)].reshape((nx, N+1))  # Reference trajectory over N+1 steps
goal_state = P[nx * (N+1):]  # Goal state

# Define weights
Q = np.diag([12.0, 12.0, 1.0])  # Weight for state tracking error
R = np.diag([12.0, 12.0])       # Weight for control effort
P_term = np.diag([10.0,10,10.0])  # Weight for goal tracking error

# Initial state constraint
st = X[:, 0]
g.append(st - ref_trajectory[:, 0])

# Formulate the optimization problem with weights
for k in range(N):
    st = X[:, k]
    con = U[:, k]
    # Cost function: weighted state deviation from trajectory + weighted control effort + terminal cost
    obj += ca.mtimes([(st - ref_trajectory[:, k]).T, Q, (st - ref_trajectory[:, k])]) + ca.mtimes([con.T, R, con]) #+ ca.mtimes([X[:,N].T,P_term,X[:N]]) TODO: fix matrix dimensions
    st_next = X[:, k+1]
    f_value = f(st, con)
    st_next_euler = st + T*f_value
    g.append(st_next - st_next_euler)

# Add goal tracking penalty to the objective function
# final_state = X[:, N]
# obj += ca.mtimes([(final_state - goal_state).T, P_goal, (final_state - goal_state)])

# Define optimization variables
OPT_variables = ca.vertcat(ca.reshape(X, nx*(N+1), 1), ca.reshape(U, nu*N, 1))

# Define NLP problem
nlp_prob = {
    'f': obj,
    'x': OPT_variables,
    'g': ca.vertcat(*g),
    'p': P
}

# Solver options
opts = {
    'ipopt': {
        'max_iter': 200,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    }
}

# Create solver
solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

# Define the bounds
lbx = []
ubx = []

# State bounds
for _ in range(N+1):
    lbx.extend([-ca.inf, -ca.inf, -ca.inf])
    ubx.extend([ca.inf, ca.inf, ca.inf])

# Control bounds
for _ in range(N):
    lbx.extend([-1.0, -np.pi/4])  # v lower bound, omega lower bound
    ubx.extend([1.0, np.pi/4])   # v upper bound, omega upper bound

lbx = ca.vertcat(*lbx)
ubx = ca.vertcat(*ubx)

# Constraint bounds (equality constraints)
lbg = ca.vertcat(*([0] * nx * (N + 1)))
ubg = lbg

# Initial state and trajectory reference states
x0 = np.array([0.0, 0.0, 0.0])
# Define a trajectory as an example
# For example, a linear trajectory from (0,0) to (5,5)
trajectory = np.linspace([0.0, 0.0, 0.0], [5.0, 5.0, 0.0], N+1)
goal = np.array([5.0, 5.0, 0.0])
# Flatten the trajectory into a single vector
p = np.concatenate((trajectory.flatten(), goal))

# Initial guess
x_init = np.zeros((nx*(N+1) + nu*N, 1))

# Solve the problem
sol = solver(x0=x_init, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

# Extract solution
sol_x = sol['x'].full().flatten()
x_solution = sol_x[:nx*(N+1)].reshape((N+1, nx))
u_solution = sol_x[nx*(N+1):].reshape((N, nu))

print("Optimal states trajectory: \n", x_solution)
print("Optimal control inputs: \n", u_solution)

# Convert ref_trajectory to numerical format
ref_trajectory_numerical = trajectory.T  # Transpose to match shape (nx, N+1)

# Plotting and animation
def animate(i):
    plt.clf()
    plt.plot(ref_trajectory_numerical[0, :], ref_trajectory_numerical[1, :], 'g--', label='Reference Trajectory')
    plt.plot(x_solution[:, 0], x_solution[:, 1], 'b-', label='Agent Path')
    plt.scatter(x_solution[i, 0], x_solution[i, 1], color='red')  # Current position of the agent
    plt.xlim(min(ref_trajectory_numerical[0, :])-1, max(ref_trajectory_numerical[0, :])+1)
    plt.ylim(min(ref_trajectory_numerical[1, :])-1, max(ref_trajectory_numerical[1, :])+1)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Agent Path and Reference Trajectory')
    plt.legend()

# Plot setup
fig = plt.figure()
ani = FuncAnimation(fig, animate, frames=N+1, interval=200, repeat=False)

plt.show()
