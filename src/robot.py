#robot.py
import numpy as np

# Robot Geometry
R       = 0.1                                   # wheel radius(m)
L       = 0.38/2                                # (m)
d       = 0.05
m_w     = 1                                     # (kg)
m_c     = 7                                     # (kg)
m       = m_c + 2*m_w                           # Eq. 31
I_w     = 0#0.005                                 # (kg-m^2)
I_m     = 0.0025                                # (kg-m^2)
I_c     = 1                                     # (kg-m^2)
I = I_c + m_c*d**2 + 2*m_w*L**2 + 2 *I_m        # Eq. 31

# Actuator characteristics
N       = 38.3                                  # gear ratio
R_a     = 0.71                                  # armature resistence (Ohms)
L_a     = 0.66e-3                               # armature inductance (H)
K_b     = 0.023                                 # emf constant (V/(rad-s))
K_t     = 0.029                                 # torque constant (N-m/A)

def v_ar(t):
    if t < 2:
        return 1
    else: 
        return 1

def v_al(t):
    if t < 2:
        return 1
    else:
        return 1
#@jit
def differential_drive_dynamics(t, x):   
    # x[0]: w_right
    # x[1]: w_left
    # x[2]: ia_right
    # x[3]: ia_left
    # x[4]: theta
    # x[5]: x 
    # x[6]: y

    # actuator modeling    
    tau_mright = K_t * x[2]     # K_t * i_aright                        # Eq. 76
    tau_mleft = K_t * x[3]      # K_t * i_aright                        # Eq. 76

    # output torque applied to the wheel
    tau = np.array([[N * tau_mright], [N * tau_mleft]], dtype=float)    # Eq. 76
    
    theta_dot = (R/(2*L)) * (x[0] - x[1])                               # Eq. 40
    V = np.array([[0, (R**2/2*L)*m_c*d*theta_dot],                      # Eq. 46
                 [-(R**2/2*L)*m_c*d*theta_dot, 0]], dtype=float)

    M = np.ndarray(shape=(2,2), dtype=float)                            # Eq. 46    
    M[0, 0] = I_w + (R**2/4*L**2)*(m*L**2 + I)
    M[0, 1] = (R**2/4*L**2)*(m*L**2 - I)
    M[1, 0] = (R**2/4*L**2)*(m*L**2 - I)
    M[1, 1] = I_w + (R**2/4*L**2)*(m*L**2 + I)

    B = np.array([[1, 0], [0, 1]], dtype=float)                         # Eq. 46
    n = np.array([[x[0]], [x[1]]], dtype=float)                         # Eq. 39
    n_dot = np.linalg.inv(M) @ (B@tau - V @ n)                          # Eq. 46

    w_m = N * np.array([[x[0]], [x[1]]], dtype=float)                   # Eq. 77
    e_a = K_b * w_m                                                     # Eq. 76
    
    i_a = np.array([[x[2]], [x[3]]], dtype=float)

    v_a = np.array([[v_ar(t)], [v_al(t)]], dtype=float)
    ia_dot = (v_a - e_a - R_a*i_a) / L_a                             # Eq. 76

    x_dot = (R/2) * np.cos(x[4]) * (x[0] + x[1])                        # Eq. 40
    y_dot = (R/2) * np.sin(x[4]) * (x[0] + x[1])                        # Eq. 40

    ret = np.zeros(7)
    ret[0] = n_dot[0,0]
    ret[1] = n_dot[1,0]
    ret[2] = ia_dot[0,0]
    ret[3] = ia_dot[1,0]
    ret[4] = theta_dot
    ret[5] = x_dot
    ret[6] = y_dot
 
    return ret
