import utils
import graphs
import numpy as np
import matplotlib.pyplot as plt

K_p = 0.28
x_g = 2
y_g = 2

csv_files_Kd = [
    ("executions/simKd-7.csv", -7, "green"),    
    ("executions/simKd-2.csv", -2, "orange"),
    ("executions/simKd-1.2.csv", -1.2, "red"),
    ("executions/simKd-0.5.csv", -0.5, "magenta"),
    ("executions/simKd-0.1.csv", -0.1, "cyan"),
]

csv_files_Kp = [
    ("executions/simKp-5.csv", -5, "red"),
    ("executions/simKp-0.2.csv", -0.2, "magenta"),
    ("executions/simKp-0.01.csv", -0.01, "cyan"),
    ("executions/simKp0.1.csv", 0.1, "orange"),     
    ("executions/simKp2500.csv", 2500, "green"),
]


if __name__ == "__main__":

    fig, ax = plt.subplots()

    for item in csv_files_Kp: 
        t, y = utils.read_model_data(item[0])
        graphs.trajectory(ax, y[0], y[1], y[2],
                    model=f"$K_p = {item[1]}$", color=item[2])

    plt.title("Top view: robot trajectory")
    plt.plot(x_g, y_g, "ro", color="black", label="Goal")
    plt.grid()
    plt.legend()
    #plt.show()
    plt.savefig(f"K_pvar.png")

    fig, ax = plt.subplots()

    for item in csv_files_Kd: 
        t, y = utils.read_model_data(item[0])
        graphs.trajectory(ax, y[0], y[1], y[2],
                    model=f"$K_d = {item[1]}$", color=item[2])

    plt.title(f"Top view: robot trajectory($K_p={K_p})$")
    plt.plot(x_g, y_g, "ro", color="black", label="Goal")
    plt.grid()
    plt.legend()
    #plt.show()
    plt.savefig(f"K_p{K_p}_K_dvar.png")
