function cenaTreinamento
%%
clear all
close all
clc
%% Initialization
Ds=0.0478;
Ls=0.0640;
thetaMax=55*pi/180;
dOrtoMax=0.0478;
%%
np=360;
hd=0.05;
tf=np*hd;
omega=2*pi/tf;
tc=0;
td=0;
id=1;
theta=0;
dOrto=0;
data=[];
%% Loading coppeliaSim remote API - client side
sim=remApi('remoteApi');
%% Closing any previously opened connections
sim.simxFinish(-1);
%% Connecting to remote coppeliaSim API server
clientID=-1;
connectCoppeliaSim()
%% Get handles
e=-1;
while (e~=0)
    [e,stripHandle]=sim.simxGetObjectHandle(clientID,'strip',sim.simx_opmode_blocking);
end
%
lsNames1=['p3dx_LS01';
    'p3dx_LS02';
    'p3dx_LS03';
    'p3dx_LS04';
    'p3dx_LS05';
    'p3dx_LS06';
    'p3dx_LS07'];
lsNames2=['p3dx_LS08';
    'p3dx_LS09';
    'p3dx_LS10';
    'p3dx_LS11';
    'p3dx_LS12';
    'p3dx_LS13';
    'p3dx_LS14'];
[lsRows,~]=size(lsNames1);
lsData1=zeros(lsRows,1);
lsData2=zeros(lsRows,1);
for i=1:lsRows
    e=-1;
    while (e~=0)
        [e,lsHandles1(i)]=sim.simxGetObjectHandle(clientID,lsNames1(i,:),sim.simx_opmode_blocking);
    end
    e=-1;
    while (e~=0)
        [e,lsHandles2(i)]=sim.simxGetObjectHandle(clientID,lsNames2(i,:),sim.simx_opmode_blocking);
    end
end
%% Starting scene simulation
e=-1;
while (e~=0)
    [e]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);
end
%%
for i=1:lsRows
    e=-1;
    while (e~=0)
        [e,detectionState,auxData,auxInfo]=sim.simxReadVisionSensor(clientID,lsHandles1(i),sim.simx_opmode_streaming);
    end
    e=-1;
    while (e~=0)
        [e,detectionState,auxData,auxInfo]=sim.simxReadVisionSensor(clientID,lsHandles2(i),sim.simx_opmode_streaming);
    end
end
t0=cputime;
while tc < tf
    tc=cputime - t0;
    if tc > td
        t(id)=tc;
        rotateAndMoveStrip()
        td=td+hd;
        id=id+1;
    end
end
%
%data=[data; tc, theta, thetaA, dOrto, dOrtoA, sensors];
t=data(:,1);
thetaR=data(:,2);
thetaE=data(:,3);
dOrtoR=data(:,4);
dOrtoE=data(:,5);
sensors=data(:, 6:size(data, 2));
figure(1)
subplot(2,1,1)
p1=plot(t, thetaR, 'b-', t, thetaE, 'r-.');
p1(1).LineWidth=2; p1(2).LineWidth=1.5;
legend('theta Reference', 'theta Estimated');

grid
subplot(2,1,2)
p2=plot(t, dOrtoR, 'b-', t, dOrtoE, 'r-.');
p2(1).LineWidth=2; p2(2).LineWidth=1.5;
legend('orto Reference', 'orto Estimated');
grid
data_save = [t, thetaR, thetaE, dOrtoR, dOrtoE, sensors];
writematrix(data_save, strcat("data/", datestr(datetime('now'), "mm.dd.yyyy HH.MM.SS"), ".csv"));
%
%% Stoping V-REP simulation
sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot_wait);
fprintf('Ending\n');
sim.simxFinish(clientID);
sim.delete();
%%    
    function connectCoppeliaSim()
        connectionAddress='127.0.0.1';
        connectionPort=19997;
        waitUntilConnected=true;
        doNotReconnectOnceDisconnected=true;
        timeOutInMs=5000;
        commThreadCycleInMs=5;
        e=1;
        while(e==1)
            [clientID]=sim.simxStart(connectionAddress,connectionPort,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs);
            if(clientID > -1)
                disp('Starting cenaTreinamento.m');
                e=0;
            else
                disp('Waiting for coppeliaSim ...');
            end
        end
    end
%%
    function rotateAndMoveStrip()
        eulerAngles=[0;0;theta];
        dOrto = (2.*dOrtoMax).*rand(1) - dOrtoMax;
        position=[4.45e-2;dOrto;1.25e-3];
        e=-1;
        while (e~=0)
            [e]=sim.simxSetObjectOrientation(clientID,stripHandle,-1,eulerAngles,sim.simx_opmode_oneshot);
        end
        e=-1;
        while (e~=0)
            [e]=sim.simxSetObjectPosition(clientID,stripHandle,-1,position,sim.simx_opmode_oneshot);
        end
        %
        [dOrtoA,thetaA, sensors]=readSensors();
        data=[data; tc, theta, thetaA, dOrto, dOrtoA, sensors];
        theta=thetaMax*sin(omega*tc);
        %
    end

%%
    function [dOrtoA,thetaA,sensors]=readSensors()
        for i=1:lsRows
            e=-1;
            while (e~=0)
                [e,~,data1,~]=sim.simxReadVisionSensor(clientID,lsHandles1(i),sim.simx_opmode_buffer);
                lsData1(i)=data1(11);
            end
            e=-1;
            while (e~=0)
                [e,~,data2,~]=sim.simxReadVisionSensor(clientID,lsHandles2(i),sim.simx_opmode_buffer);
                lsData2(i)=data2(11);
            end
        end
        %
        sensors = [lsData1', lsData2'];
        m1=min(lsData1);
        m2=min(lsData2);
        
        i1=min(find(lsData1 == m1));
        i2=7+min(find(lsData2 == m2));

        p1=(i1-4);
        p2=(i2-11);
        thetaA=atan2((p2-p1)*Ds,Ls);
        dOrtoA=(p1+p2)*Ds;
    end
%%
end
