import robot
import graphs
import utils

import time
import timeit
import argparse
import numpy as np
import scene as sim_scene
from scipy.integrate import solve_ivp

# initial values
iv = np.zeros(7)
iv[0] = 0           # right wheel speed
iv[1] = 0           # left wheel speed
iv[2] = 0           # right wheel armature current
iv[3] = 0           # left wheel armature current
iv[4] = 0           # robot yaw angle (theta)
iv[5] = -2          # robot x position
iv[6] = -2          # robot y position

plot_choices = ["a", "p", "t", "tj", "none", "all"]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--f", type=str, default="",
                        help="file containing past execution data")
    parser.add_argument ("--tf", type=float, required=True,
                        help="final integration time")

    plot_help = "plot options, allowed values are: " + ",".join(plot_choices)
    plot_help += """
a:   armature current and voltage
p:   robot position and wheel velocities
t:   torque exerted on both wheels
tj:  robot trajectory
all: display all available plots
none: no plot is displayed
"""

    parser.add_argument("--plot", nargs="+", type=str, choices=plot_choices,
                        default="all",
                        help=plot_help, metavar="")

    args = parser.parse_args()

    if "all" in args.plot:
        args.plot = plot_choices[:-2]
    elif "none" in args.plot:
        args.plot = []

    t_span = [0, args.tf]

    sol_t = None
    sol_y = None
   
    if args.f == "":
        print(f"Solving robot dynamics for given initial conditions with time span = (0, {t_span[1]})...")
        t0 = timeit.default_timer()
        sol = solve_ivp(robot.differential_drive_dynamics, t_span, iv)
        tf = timeit.default_timer()

        sol_t = sol.t
        sol_y = sol.y

        print(f"Computation finished within {tf-t0:.2f} second(s).")
        #
        file_name = time.strftime("%d_%m_%Y_%H_%M_%S", time.localtime(int(time.time())))
        file_path = f"../executions/{file_name}.csv"

        print(f"Saving execution on {file_path}...")
        utils.save_model_data(file_path, sol_t, sol_y)
        #
    else:
        print(f"Loading dynamics data from {args.f}")
        sol_t, sol_y = utils.read_model_data(args.f)

        if t_span[1] > sol_t[-1]:
            print(f"[WARNING] final time input({t_span[1]}s) was greater than stored execution time({sol_t[-1]}s)."
                 f"Running with tf = {sol_t[-1]} instead.")

            t_span[1] = sol_t[-1]


    print("Running CoppeliaSim simulation...")
    t0 = timeit.default_timer()
    sim = sim_scene.run(sol_t, iv, sol_y[0], sol_y[1])
    tf = timeit.default_timer()

    print(f"Simulation finished within {tf-t0:.2f} second(s).")
    #
    print("Ploting results...")
    theta   = [sol_y[4], sim["theta"]]
    x       = [sol_y[5], sim["x"]]
    y       = [sol_y[6], sim["y"]]

    tau_r = robot.N * robot.K_t * sol_y[3]                      # Eq. 76
    tau_l = robot.N * robot.K_t * sol_y[4]                      # Eq. 76

    v_ar = [robot.v_ar(t) for t in sol_t]
    v_al = [robot.v_ar(t) for t in sol_t]

    if "a" in args.plot:
        block = args.plot[-1] == "a"
        graphs.armature(sol_t, [sol_y[3], sol_y[4]], [v_ar, v_al], blocking=block)

    if "p" in args.plot:
        block = args.plot[-1] == "p"
        graphs.position(sol_t, sol_y[0], sol_y[1], sol_y[4], sol_y[5], sol_y[6], "Robot position(model)")
        graphs.position(sim["t"], sim["w_r"], sim["w_l"], sim["theta"], 
                        sim["x"], sim["y"], "Robot position(simulation)", blocking=block)

    if "t" in args.plot:
        block = args.plot[-1] == "t"
        graphs.torque([sol_t, sim["t"]], [tau_r, sim["tau_r"]], [tau_l, sim["tau_l"]], blocking=block)

    if "tj" in args.plot:
        block = args.plot[-1] == "tj"
        graphs.trajectory(x, y, theta, blocking=block)
