import robot
import timeit
import numpy as np
import graphs
import robot
import scene as sim_scene
from scipy.integrate import solve_ivp

t_span = (0, 10)

# initial values
iv = np.zeros(7)
iv[0] = 0           # right wheel speed
iv[1] = 0           # left wheel speed
iv[2] = 0           # right wheel armature current
iv[3] = 0           # left wheel armature current
iv[4] = 0           # robot yaw angle (theta)
iv[5] = -2          # robot x position
iv[6] = -2          # robot y position

if __name__ == "__main__":
    print(f"Solving robot dynamics for given initial conditions...")
    t0 = timeit.default_timer()
    sol = solve_ivp(robot.differential_drive_dynamics, t_span, iv, dense_output=True)
    tf = timeit.default_timer()

    print(f"Computation finished within {tf-t0:.2f} second(s).")
    #
    print("Running CoppeliaSim simulation...")
    t0 = timeit.default_timer()
    sim = sim_scene.run(sol.t, iv, sol.y[0], sol.y[1])
    tf = timeit.default_timer()

    print(f"Simulation finished within {tf-t0:.2f} second(s).")
    #
    print("Ploting results...")
    theta   = [sol.y[4], sim["theta"]]
    x       = [sol.y[5], sim["x"]]
    y       = [sol.y[6], sim["y"]]

    tau_r = robot.N * robot.K_t * sol.y[3]                      # Eq. 76
    tau_l = robot.N * robot.K_t * sol.y[4]                      # Eq. 76

    v_ar = [robot.v_ar(t) for t in sol.t]
    v_al = [robot.v_ar(t) for t in sol.t]
    #graphs.armature(sol.t, [sol.y[3], sol.y[4]], [v_ar, v_al])
    graphs.position(sol.t, sol.y[0], sol.y[1], sol.y[4], sol.y[5], sol.y[6], "Robot position(model)")

    graphs.position(sim["t"], sim["w_r"], sim["w_l"], sim["theta"], 
                    sim["x"], sim["y"], "Robot position(simulation)")

    graphs.torque([sol.t, sim["t"]], [tau_r, sim["tau_r"]], [tau_l, sim["tau_l"]], blocking=True)

    #graphs.trajectory(x, y, theta, blocking=True)
