#graphs.py

import numpy as np
import matplotlib.pyplot as plt

def current(t, out, blocking=False):
    plt.figure()
    plt.title("Armature current")
    plt.plot(t, out[2], label=r"$i_{a_R}$")
    plt.plot(t, out[3], label=r"$i_{a_L}$")
    plt.legend()
    plt.xlabel(r"$t [s]$")
    plt.ylabel(r"$i_a [A]$")
    plt.grid()
    plt.tight_layout()
    plt.show(block=blocking)

def position(t, out, blocking=False):
    plt.figure()
    axs = plt.subplot(2, 1, 1)
    axs.title.set_text("Robot position")

    plt.plot(t, out[5], label=r"$x [m]$")
    plt.plot(t, out[6], label=r"$y [m]$")
    plt.plot(t, out[4], label=r"$\theta [º]$", linewidth=2)    
    plt.legend()
    plt.xlabel(r"$t [s]$")
    plt.grid()

    axs = plt.subplot(2, 1, 2)
    axs.title.set_text("Velocity of each wheel")

    plt.plot(t, out[0], label=r"$\dot{\phi_R}$")
    plt.plot(t, out[1], label=r"$\dot{\phi_L}$")
    plt.legend()
    plt.xlabel(r"$t\;[s]$")
    plt.ylabel(r"$\dot{\phi} [rad/s]$")
    plt.grid()
    plt.tight_layout()
    plt.show(block=blocking)

def draw_robot(x, y, q, s, h):
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
    h.plot(X, Y, 'r-')

def trajectory(y, blocking=False):
    fig, ax = plt.subplots()
    plt.title("Top view: robot trajectory")
    ax.axis("equal")
    ax.plot(y[5], y[6], color="blue", linestyle="dashed", linewidth=1)

    plt.xlabel(r"x [m]")
    plt.ylabel(r"y [m]")
    plt.grid()    
    plt.show(block=False)

    for i in range(0, len(y[5]) - 1, int(round(len(y[5]) / 20))):
        draw_robot(y[5][i], y[6][i], y[4][i], 0.01, ax)

    plt.tight_layout()
    plt.show(block=blocking)
