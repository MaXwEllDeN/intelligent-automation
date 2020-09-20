#graphs.py

import numpy as np
import matplotlib.pyplot as plt

def armature(t, i_a, v_a, blocking=False):
    plt.figure()
    axs = plt.subplot(2, 1, 1)
    axs.title.set_text("Armature current")

    plt.plot(t, i_a[0], label=r"$i_{a_R}$")
    plt.plot(t, i_a[1], label=r"$i_{a_L}$")
    plt.legend()
    plt.xlabel(r"$t [s]$")
    plt.ylabel(r"$i_a [A]$")
    plt.grid()

    axs = plt.subplot(2, 1, 2)
    axs.title.set_text("Armature Voltage")

    plt.plot(t, v_a[0], label=r"$v_{a_R}$")
    plt.plot(t, v_a[1], label=r"$v_{a_L}$")
    plt.legend()
    plt.xlabel(r"$t\;[s]$")
    plt.ylabel(r"$Voltage [V]$")
    plt.grid()

    plt.tight_layout()
    plt.show(block=blocking)

def torque(t, tau_r, tau_l, blocking=False):
    plt.figure()
    axs = plt.subplot(2, 1, 1)
    axs.title.set_text("Right wheel torque")

    plt.plot(t[0], tau_r[0], label=r"Model")
    plt.plot(t[1], tau_r[1], label=r"Simulation")
    plt.legend()
    plt.xlabel(r"$t [s]$")
    plt.ylabel(r"$\tau\;[N \cdot m]$")
    plt.grid()

    axs = plt.subplot(2, 1, 2)
    axs.title.set_text("Left wheel torque")

    plt.plot(t[0], tau_l[0], label=r"Model")
    plt.plot(t[1], tau_l[1], label=r"Simulation")
    plt.legend()
    plt.xlabel(r"$t [s]$")
    plt.ylabel(r"$\tau\;[N \cdot m]$")
    plt.grid()

    plt.tight_layout()
    plt.show(block=blocking)

def position(t, w_r, w_l, theta, x, y, title, blocking=False):
    plt.figure()
    axs = plt.subplot(2, 1, 1)
    axs.title.set_text(title)

    plt.plot(t, x, label=r"$x [m]$")
    plt.plot(t, y, label=r"$y [m]$")
    plt.plot(t, theta, label=r"$\theta [º]$", linewidth=2)    
    plt.legend()
    plt.xlabel(r"$t [s]$")
    plt.grid()

    axs = plt.subplot(2, 1, 2)
    axs.title.set_text("Velocity of each wheel")

    plt.plot(t, w_r, label=r"$\dot{\phi_R}$")
    plt.plot(t, w_l, label=r"$\dot{\phi_L}$")
    plt.legend()
    plt.xlabel(r"$t\;[s]$")
    plt.ylabel(r"$\dot{\phi} [rad/s]$")
    plt.grid()

    plt.tight_layout()
    plt.show(block=blocking)

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

def trajectory(x, y, theta, blocking=False):
    fig, ax = plt.subplots()
    plt.title("Top view: robot trajectory")
    ax.axis("equal")

    ax.plot(x[0], y[0], color="blue", linestyle="dashed", linewidth=1)
    ax.plot(x[1], y[1], color="black", linestyle="dashed", linewidth=1)

    plt.xlabel(r"x [m]")
    plt.ylabel(r"y [m]")
    plt.grid()
    plt.show(block=False)

    for i in range(0, len(x[0]) - 1, int(round(len(x[0]) / 20))):
        last_robot_model = draw_robot(x[0][i], y[0][i], theta[0][i], 0.01, ax, "red")

    ax.lines[-1].set_label("Model")

    for i in range(0, len(x[1]) - 1, int(round(len(x[1]) / 20))):
        last_robot_sim = draw_robot(x[1][i], y[1][i], theta[1][i], 0.01, ax, "green")

    ax.lines[-1].set_label("Simulation")

    plt.tight_layout()
    plt.legend()

    plt.show(block=blocking)
