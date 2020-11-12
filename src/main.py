# This script shall be executed with the scene "ControleUniciclo.ttt"

import os
import utils
import models
import graphs
import scene
import numpy as np

from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt

SAVE_PNG = False
final_time = 500

# Stop distance
stop_distance = 5e-3
time_step = 50e-3

# initial values P
iv_p = np.zeros(3)
iv_p[0] = models.x_0       # x
iv_p[1] = models.y_0       # y
iv_p[2] = models.phi_0     # phi

# initial values PD
iv_pd = np.zeros(4)
iv_pd[0] = models.x_0       # x
iv_pd[1] = models.y_0       # y
iv_pd[2] = models.phi_0     # phi
iv_pd[3] = models.z_0       # z

if __name__ == "__main__":
    t_span = [0, final_time]
    eval_at = np.arange(t_span[0], t_span[1], time_step)

    test_models = []
    #test_models.append(("P", models.ddmr_p_nonlinear, iv_p, models.K_p, 0, True))
    test_models.append(("PD", models.ddmr_pd_nonlinear, iv_pd, models.K_p, models.K_d, True))

    for item in test_models:
        item_name = item[0]
        item_ode = item[1]
        item_iv = item[2]
        item_kp = item[3]
        item_kd = item[4]
        run_sim = item[5]

        print(f"Running for: {item_name}")

        sol = solve_ivp(item_ode, t_span, item_iv, t_eval=eval_at)
        sol_t = sol.t
        sol_y = sol.y

        distances = np.array(np.sqrt((2 - sol_y[1])**2 + (2-sol_y[0])**2))
        result = np.where(distances <= stop_distance)

        index = len(sol_t)

        if len(result[0]) != 0:
            index = result[0][0]

        t = sol_t[0:index]
        x = sol_y[0][0:index]
        y = sol_y[1][0:index]
        phi = sol_y[2][0:index]

        sim_result = None
        print(f"Model finished after {max(t):.2f} seconds.")

        if run_sim:
            sim_result = scene.run(sol_t, models.x_0, models.y_0, models.phi_0, item_kp, item_kd)
            finished_at  = max(sim_result["t"])
            print(f"Simulation finished after {finished_at:.2f} seconds.")
            sim_y = []
            sim_y.append(sim_result["x"])
            sim_y.append(sim_result["y"])
            sim_y.append(sim_result["phi"])

            file_name = f"simKp{item_kp}.csv"
            if item_name == "PD":
                file_name = f"simKd{item_kd}.csv"

            utils.save_model_data(file_name, sim_result["t"], sim_y)

        graphs.position(t, phi, x, y, title=f"Model {item_name}")
        
        if SAVE_PNG:
            plt.savefig(f"{item_name}_pos_model.png")
        else:
            plt.show(block=False)

        if run_sim:
            graphs.position(sim_result["t"], sim_result["phi"], sim_result["x"],
                            sim_result["y"], title=f"Simulation {item_name}")
            if SAVE_PNG:
                plt.savefig(f"{item_name}_pos_sim.png")
            else:
                plt.show(block=False)

        fig, ax = plt.subplots() 
        graphs.trajectory(ax, x, y, phi, model="Model", color="red")

        if run_sim:        
            graphs.trajectory(ax, sim_result["x"], sim_result["y"], sim_result["phi"],
                            model="Simulation", color="green")

        plt.title(f"Top view: robot trajectory({item_name})")
        plt.grid()
        plt.legend()

        if SAVE_PNG:
            plt.savefig(f"{item_name}_trajectory.png")
            print(f"Plots saved at {os.getcwd()}")            
        else:
            plt.show()
