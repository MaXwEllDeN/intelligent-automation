#
# coppeliaSim
import sys
import sim, simConst

import numpy as np

import matplotlib.pyplot as plt
import graphs

connectionAddress = "127.0.0.1"
connectionPort = 19997
waitUntilConnected = True
doNotReconnectOnceDisconnected = True
timeOutInMs = 5000
commThreadCycleInMs = 5

def run(in_t, iv, in_w_r, in_w_l):
    time_step = 50e-3

    # Resampling model data
    t = np.arange(0, max(in_t) + time_step, time_step)    
    w_r = np.interp(t, in_t, in_w_r)
    w_l = np.interp(t, in_t, in_w_l)

    clientID = -1
    sim.simxFinish(clientID)

    res = -1
    while (res < 0):
        clientID = sim.simxStart(connectionAddress, connectionPort, waitUntilConnected, 
                                doNotReconnectOnceDisconnected, timeOutInMs, commThreadCycleInMs)

        if (clientID > -1):
            res = 0
        else:
            print('Waiting for coppeliaSim ..');

    print("\t -> Connected to remote API server.")

    # Getting robot handle
    res = -1
    while (res != sim.simx_return_ok):
        res, rob = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)

    _, leftMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, rightMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

    sim.simxSynchronous(clientID, True)
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, time_step, sim.simx_opmode_blocking)

    # Defining coppeliaSim client side parameters
    xp = np.zeros(len(t))
    yp = np.zeros(len(t))
    thetap = np.zeros(len(t))
    tau_r = np.zeros(len(t))
    tau_l = np.zeros(len(t))
    w_rp = np.zeros(len(t))
    w_lp = np.zeros(len(t))

    # Get initial position and orientation
    res = -1
    while (res != sim.simx_return_ok):
        res, robPosI = sim.simxGetObjectPosition(clientID, rob, -1, sim.simx_opmode_streaming)

    robPosI[0] = iv[5]
    robPosI[1] = iv[6]

    res = -1
    while (res != sim.simx_return_ok):
        res, robOriI = sim.simxGetObjectOrientation(clientID, rob, -1, sim.simx_opmode_streaming)

    robOriI[2] = iv[4]

    sim.simxSetObjectPosition(clientID, rob, -1, robPosI, sim.simx_opmode_oneshot)
    sim.simxSetObjectOrientation(clientID, rob, -1, robOriI, sim.simx_opmode_oneshot)

    #
    # As recommeded by the documentation, the first simxGetJointForce call must be done with
    # simx_opmode_streaming mode

    res = -1
    while (res != sim.simx_return_ok):
        res, _ = sim.simxGetJointForce(clientID, rightMotor, sim.simx_opmode_streaming)

    res = -1
    while (res != sim.simx_return_ok):
        res, _ = sim.simxGetJointForce(clientID, leftMotor, sim.simx_opmode_streaming)

    #
    # As recommeded by the documentation, the first simxGetObjectVelocity call must be done with
    # simx_opmode_streaming mode

    res = -1
    while (res != sim.simx_return_ok):
        res, _, _ = sim.simxGetObjectVelocity(clientID, rightMotor, sim.simx_opmode_streaming)

    res = -1
    while (res != sim.simx_return_ok):
        res, _, _ = sim.simxGetObjectVelocity(clientID, leftMotor, sim.simx_opmode_streaming)

    #
    # Starting coppeliaSim simulation
    res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    # Main control loop - coppeliaSim client side

    for id in range(0, len(t + 1)):
        # Measuring
        _, robPos = sim.simxGetObjectPosition(clientID, rob, -1, sim.simx_opmode_buffer)
        _, robOri = sim.simxGetObjectOrientation(clientID, rob, -1, sim.simx_opmode_buffer)

        _, robRightMotorTorque = sim.simxGetJointForce(clientID, rightMotor, sim.simx_opmode_buffer)
        _, robLeftMotorTorque = sim.simxGetJointForce(clientID, leftMotor, sim.simx_opmode_buffer)

        _, _, robRightMotorVelocity = sim.simxGetObjectVelocity(clientID, rightMotor, sim.simx_opmode_buffer)
        _, _, robLeftMotorVelocity = sim.simxGetObjectVelocity(clientID, leftMotor, sim.simx_opmode_buffer)

        # Actuating
        sim.simxSetJointTargetVelocity(clientID, rightMotor, w_r[id], sim.simx_opmode_oneshot)        
        sim.simxSetJointTargetVelocity(clientID, leftMotor, w_l[id], sim.simx_opmode_oneshot)

        # Saving
        xp[id] = robPos[0]
        yp[id] = robPos[1]
        thetap[id] = robOri[2]
        tau_r[id] = robRightMotorTorque
        tau_l[id] = robLeftMotorTorque
        w_rp[id] = robRightMotorVelocity[0]
        w_lp[id] = robLeftMotorVelocity[0]

        sim.simxSynchronousTrigger(clientID)

    # Stoping coppeliaSim simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxFinish(clientID)

    res = {
        "t": t,
        "w_r": w_rp,
        "w_l": w_lp,
        "theta": thetap,
        "tau_r": tau_r,
        "tau_l": tau_l,
        "x": xp,
        "y": yp
    }

    return res
