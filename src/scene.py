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

def run(t, iv_x, iv_y, iv_phi, K_p, K_d):
    clientID = -1
    sim.simxFinish(clientID)

    res = -1
    while (res < 0):
        clientID = sim.simxStart(connectionAddress, connectionPort, waitUntilConnected, 
                                doNotReconnectOnceDisconnected, timeOutInMs, commThreadCycleInMs)

        if (clientID > -1):
            res = 0
        else:
            print('Waiting for coppeliaSim ..')

    print("\t -> Connected to remote API server.")

    # Getting robot handle
    res = -1
    while (res != sim.simx_return_ok):
        res, rob = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)

    sim.simxSynchronous(clientID, True)

    # Defining coppeliaSim client side parameters
    ret_t = []
    xp = []
    yp = []
    phip = []

    # Setting PD parameters:
    res = -1
    while (res != sim.simx_return_ok):
        res = sim.simxSetStringSignal (clientID, "K_p", str(K_p), sim.simx_opmode_oneshot)

    res = -1
    while (res != sim.simx_return_ok):
        res = sim.simxSetStringSignal (clientID, "K_d", str(K_d), sim.simx_opmode_oneshot)

    # Get initial position and orientation
    res = -1
    while (res != sim.simx_return_ok):
        res, robPosI = sim.simxGetObjectPosition(clientID, rob, -1, sim.simx_opmode_streaming)

    robPosI[0] = iv_x
    robPosI[1] = iv_y

    res = -1
    while (res != sim.simx_return_ok):
        res, robOriI = sim.simxGetObjectOrientation(clientID, rob, -1, sim.simx_opmode_streaming)

    robOriI[2] = iv_phi

    sim.simxSetObjectPosition(clientID, rob, -1, robPosI, sim.simx_opmode_oneshot)
    sim.simxSetObjectOrientation(clientID, rob, -1, robOriI, sim.simx_opmode_oneshot)

    #
    # Starting coppeliaSim simulation
    res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    # Main control loop - coppeliaSim client side

    lastInfo = 0
    for id in range(0, len(t)):
        # Measuring
        _, robPos = sim.simxGetObjectPosition(clientID, rob, -1, sim.simx_opmode_buffer)
        _, robOri = sim.simxGetObjectOrientation(clientID, rob, -1, sim.simx_opmode_buffer)

        _, info = sim.simxGetInMessageInfo(clientID, sim.simx_headeroffset_server_state)

        if info == 0 and lastInfo == 1:
            break

        lastInfo = info

        # Saving
        ret_t.append(t[id])
        xp.append(robPos[0])
        yp.append(robPos[1])
        phip.append(robOri[2])

        sim.simxSynchronousTrigger(clientID)


    # Stoping coppeliaSim simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxFinish(clientID)

    res = {
        "t": np.array(ret_t),
        "phi": np.array(phip),
        "x": np.array(xp),
        "y": np.array(yp)
    }

    return res
