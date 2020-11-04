import utils
import graphs
import numpy as np
import matplotlib.pyplot as plt

K_p = 0.28
x_g = 2
y_g = 2

csv_files = [
    ("executions/sim_m1.5.csv", -1.5, "green"),
    ("executions/sim_m1.7.csv", -1.7, "red"),
    ("executions/sim_m0.9.csv", -0.9, "orange"),    
    ("executions/sim_m2.csv", -2, "magenta"),
    ("executions/sim_p15.csv", 15, "cyan"),

]

if __name__ == "__main__":

    fig, ax = plt.subplots()

    for item in csv_files: 
        t, y = utils.read_model_data(item[0])
        graphs.trajectory(ax, y[0], y[1], y[2],
                    model=f"$K_d = {item[1]}$", color=item[2])

    plt.title(f"Top view: robot trajectory($K_p={K_p})$")
    plt.plot(x_g, y_g, "ro", color="black", label="Goal")
    plt.grid()
    plt.legend()    
    #plt.show()
    plt.savefig(f"K_p{K_p}_K_dvar.png")