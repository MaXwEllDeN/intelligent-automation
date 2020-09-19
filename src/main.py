import robot
import timeit
import numpy as np
import graphs
from scipy.integrate import solve_ivp

t_span = (0, 5)

# initial values
iv = np.zeros(7)
iv[0] = 0           # right wheel speed
iv[1] = 0           # left wheel speed
iv[2] = 0           # right wheel armature current
iv[3] = 0           # left wheel armature current
iv[4] = 0           # robot orientation (theta)
iv[5] = -2          # robot x position
iv[6] = -2          # robot y position

if __name__ == "__main__":
    print(f"Solving robot dynamics for given initial conditions...")

    t0 = timeit.default_timer()
    sol = solve_ivp(robot.differential_drive_dynamics, t_span, iv, dense_output=True)
    tf = timeit.default_timer()

    print(f"Computation finished within {tf-t0:.2f} second(s).")

    print("Ploting results...")
    #graphs.current(sol.t, sol.y)
    graphs.position(sol.t, sol.y)
    graphs.trajectory(sol.y, blocking=True)
