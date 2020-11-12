import numpy as np

# Initial values
v_0 = 0.2
x_0 = 0
y_0 = 0
z_0 = 0
phi_0 = np.pi/4

# Goal
x_g = 2
y_g = 2

# Controller
K_p = 0.28
K_d = 15

def ddmr_p_nonlinear(t, x):
    # x[0]: x_pos
    # x[1]: y_pos
    # x[2]: phi

    phi = x[2]
    phi_d = np.arctan((y_g - x[1]) / (x_g - x[0]))
    e = phi_d - phi

    states = np.zeros(3)
    states[0] = v_0 * np.cos(phi)
    states[1] = v_0 * np.sin(phi)
    states[2] = K_p * e

    return states

A = np.matrix([[0, 0, -v_0*np.sin(phi_0)],
                [0, 0, v_0*np.cos(phi_0)],
                [-K_p*(y_0 - y_g)/((y_0 - y_g)**2 + (x_0 - x_g)**2)**2,
                K_p*(x_0 - x_g)/((y_0 - y_g)**2 + (x_0 - x_g)**2), -K_p],
])

B = np.matrix([[0], [0], [K_p]])

def ddmr_pd_nonlinear(t, x):
    # x[0]: x_pos
    # x[1]: y_pos
    # x[2]: phi
    # x[3]: x

    c = 1
    z = x[3]
    phi = x[2]
    phi_d = np.arctan((y_g - x[1]) / (x_g - x[0]))
    e = phi_d - phi

    states = np.zeros(4)
    states[0] = v_0 * np.cos(phi)
    states[1] = v_0 * np.sin(phi)
    states[2] = -z*(c**2)*K_d + e * (K_p +c*K_d)    
    states[3] = e - c * z
    return states
