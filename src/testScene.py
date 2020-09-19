#
# coppeliaSim
import sys
import sim, simConst

# coppeliaSim
import timeit, numpy, scipy
import matplotlib.pyplot as plt
#
def drawRobot(x,y,q,s,h):
    p=numpy.zeros(36).reshape(12,3)
    p[0,:]=[1,1/7,1/s]
    p[1,:]=[-3/7,1,1/s]
    p[2,:]=[-5/7,6/7,1/s]
    p[3,:]=[-5/7,5/7,1/s]
    p[4,:]=[-3/7,2/7,1/s]
    p[5,:]=[-3/7,0,1/s]
    p[6,:]=[-3/7,-2/7,1/s]
    p[7,:]=[-5/7,-5/7,1/s]
    p[8,:]=[-5/7,-6/7,1/s]
    p[9,:]=[-3/7,-1,1/s]
    p[10,:]=[1,-1/7,1/s]
    p[11,:]=[1,1/7,1/s]
    #
    p=s*p
    #
    r=numpy.zeros(6).reshape(3,2)
    r[0,:]=[numpy.cos(q),numpy.sin(q)]
    r[1,:]=[-numpy.sin(q),numpy.cos(q)]
    r[2,:]=[x,y]
    #
    p=numpy.dot(p,r)
    X=p[:,0]
    Y=p[:,1]
    h.plot(X,Y,'r-')


connectionAddress = "127.0.0.1"
connectionPort = 19997
waitUntilConnected = True
doNotReconnectOnceDisconnected = True
timeOutInMs = 5000
commThreadCycleInMs = 5

clientID = -1
sim.simxFinish(clientID)

res = -1
while (res < 0):
    clientID = sim.simxStart(connectionAddress, connectionPort, waitUntilConnected, 
                            doNotReconnectOnceDisconnected, timeOutInMs, commThreadCycleInMs)

    if (clientID > -1):
        print("Starting cenaTeste.py...")
        res = 0
    else:
        print('Waiting for coppeliaSim ..');

print("Connected to remote API server.")


# Getting robot handle
res = -1
while (res != sim.simx_return_ok):
    res, rob = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)

res, leftMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
res, rightMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

# P3DX dimensions
D = 195e-3
R = D/2
L = 331e-3

# Defining coppeliaSim client side parameters
np = 100
hd = 50e-3
tf = np*hd
tc = 0
td = 0
id = 0

t = numpy.zeros(np)
up = numpy.zeros(np)
om = numpy.zeros(np)
xp = numpy.zeros(np)
yp = numpy.zeros(np)
fp = numpy.zeros(np)

# Starting coppeliaSim simulation
res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
# Get initial pose = [position orientation]
res = -1
while (res != sim.simx_return_ok):
    res, robPosI = sim.simxGetObjectPosition(clientID, rob, -1, sim.simx_opmode_streaming)


res = -1
while (res != sim.simx_return_ok):
    res, robOriI = sim.simxGetObjectOrientation(clientID, rob, -1, sim.simx_opmode_streaming)

t0 = timeit.default_timer()

# Main control loop - coppeliaSim client side
while (tc < tf):   
    # Current sampling instant
    if (tc > td):
        t[id] = tc
        # Measuring
        _, robPos = sim.simxGetObjectPosition(clientID, rob, -1, sim.simx_opmode_buffer)
        _, robOri = sim.simxGetObjectOrientation(clientID, rob, -1, sim.simx_opmode_buffer)

        # Controlling
        if (tc < tf/2):
            Ups = 0.2
            Ome = 0.0
        else:
            Ups = 0.2
            Ome = 0.1
        #
        LeftVel = (2*Ups-Ome*L)/(2*R)
        RightVel = (2*Ups+Ome*L)/(2*R)

        # Actuating
        sim.simxSetJointTargetVelocity(clientID, leftMotor, LeftVel, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, rightMotor, RightVel, sim.simx_opmode_oneshot)

        # Saving
        up[id] = Ups
        om[id] = Ome
        xp[id] = robPos[0]
        yp[id] = robPos[1]
        fp[id] = robOri[2]

        # Next sampling instant
        td = td + hd
        id = id + 1

    tc = timeit.default_timer() - t0

# Stoping coppeliaSim simulation
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
print("Ending cenaTeste.py")
sim.simxFinish(clientID)

# Plotting results
plt.figure(1)
plt.subplot(2,1,1)
plt.plot(t, xp, label="x_p(t)", linewidth=2)
plt.plot(t, yp, label="y_p(t)", linewidth=2)
plt.plot(t, fp, label="phi_p(t)", linewidth=2)
plt.legend()
plt.xlabel("t [s]")
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(t, up, label="upsilon(t)", linewidth=2)
plt.plot(t, om, label="omega(t)", linewidth=2)
plt.legend()
plt.xlabel("t [s]")
plt.grid()
plt.show()

fig, ax = plt.subplots()
ax.axis("equal")
ax.plot(xp, yp, color="blue",linestyle="dashed",linewidth=1)
plt.grid()
plt.title("Top view: robot trajectory")
plt.xlabel("x, m")
plt.ylabel("y, m")
plt.show(block=False)
#
for i in range(0,len(xp)-1,int(round(len(xp)/20))):
    drawRobot(xp[i],yp[i],fp[i],0.01,ax)
#
plt.show()
