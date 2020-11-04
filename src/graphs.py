#graphs.py

import numpy as np
import matplotlib.pyplot as plt

def position(t, phi, x, y, title):
    fig, ax1 = plt.subplots()
    plt.title(f"Robot Position({title})")

    ax1.plot(t, x, label=r"$x [m]$", color="green")
    ax1.plot(t, y, label=r"$y [m]$", color="blue")

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    ax2.set_ylabel("$\phi[º]$")
    ax2.plot(t, phi * 180/np.pi, label=r"$\phi [º]$", color="red", linewidth=2)
    ax2.legend(loc="lower right")

    ax1.legend(loc="upper right")
    ax1.set_ylabel(r"$x,\; y [s]$")
    ax1.set_xlabel(r"$t [s]$")
    ax1.grid()

def draw_robot(x, y, q, s, h, color):
    p = np.zeros(36).reshape(12,3)
    p[0,:] = [1,1/7,1/s]
    p[1,:] = [-3/7,1,1/s]
    p[2,:] = [-5/7,6/7,1/s]
    p[3,:] =[-5/7,5/7,1/s]
    p[4,:] = [-3/7,2/7,1/s]
    p[5,:] = [-3/7,0,1/s]
    p[6,:] = [-3/7,-2/7,1/s]
    p[7,:] = [-5/7,-5/7,1/s]
    p[8,:] = [-5/7,-6/7,1/s]
    p[9,:] = [-3/7,-1,1/s]
    p[10,:] = [1,-1/7,1/s]
    p[11,:] = [1,1/7,1/s]

    p = s*p
    #
    r = np.zeros(6).reshape(3,2)
    r[0,:] = [np.cos(q), np.sin(q)]
    r[1,:] = [-np.sin(q), np.cos(q)]
    r[2,:] = [x, y]
    #
    p = np.dot(p, r)
    X = p[:, 0]
    Y = p[:, 1]

    h.plot(X, Y, "-", color=color)

def trajectory(ax, x, y, phi, model, color):
    ax.axis("equal")

    ax.plot(x, y, color="blue", linestyle="dashed", linewidth=1)

    plt.xlabel(r"x [m]")
    plt.ylabel(r"y [m]")

    for i in range(0, len(x) - 1, int(round(len(x) / 20))):
        draw_robot(x[i], y[i], phi[i], 0.01, ax, color)

    ax.lines[-1].set_label("Model")
    ax.lines[-1].set_label("Simulation")
    ax.lines[-1].set_label(model)
