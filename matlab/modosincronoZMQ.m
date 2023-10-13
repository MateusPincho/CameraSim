% Make sure to have the add-on "ZMQ remote API" running in CoppeliaSim
%
clear all
close all
clc

% P3DX dimensions
D=195e-3;
R=D/2;
L=381e-3;

Rc=1.0;

fprintf('Program started\n')

client = RemoteAPIClient();
sim = client.getObject('sim');

% When simulation is not running, ZMQ message handling could be a bit
% slow, since the idle loop runs at 8 Hz by default. So let's make
% sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps);
sim.setInt32Param(sim.intparam_idle_fps, 0);

% Load CoppeliaSim scene
SceneFolder='/home/mateus/Documentos/CameraSim/';
SceneName='modosincronoZMQ.ttt';
fileScene=[SceneFolder SceneName];
sim.loadScene(fileScene);

% Get the simulation time step
dt=sim.getSimulationTimeStep();

np=500;
tf=np*dt;
t=zeros(np,1);  
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);
up=zeros(np,1);
om=zeros(np,1);
vr=zeros(np,1);
vl=zeros(np,1);


PioneerP3DX=sim.getObject('/PioneerP3DX');
rightMotor=sim.getObject('/PioneerP3DX/rightMotor');
leftMotor=sim.getObject('/PioneerP3DX/leftMotor');

% Run a simulation in stepping mode:
client.setStepping(true);

sim.startSimulation();
id=0;
while 1
    ts = sim.getSimulationTime();
    if ts >= tf; break; end
    id=id+1;
    t(id)=ts;
    P3DXPos=sim.getObjectPosition(PioneerP3DX,-1);
    P3DXOri=sim.getObjectOrientation(PioneerP3DX,-1);
    xp(id)=cell2mat(P3DXPos(1,1));
    yp(id)=cell2mat(P3DXPos(1,2));
    fp(id)=cell2mat(P3DXOri(1,3));
    Ome=0.5;
    if ts > tf/2,
        Rc=0.5;
    end
    Vr=Ome*(Rc+L/2);Vl=Ome*(Rc-L/2);
    rightVel=Vr/R;leftVel=Vl/R;
    Ups=R*(leftVel+rightVel)/2;
    up(id)=Ups;
    om(id)=Ome;
%     leftVel=(2*Ups-Ome*L)/(2*R); rightVel=(2*Ups+Ome*L)/(2*R);
    vr(id)=rightVel;
    vl(id)=leftVel;
    
    sim.setJointTargetVelocity(leftMotor,leftVel);
    sim.setJointTargetVelocity(rightMotor,rightVel);
    client.step();  % triggers next simulation step
end
sim.stopSimulation();
% To make sure we really stopped:
while sim.getSimulationState() ~= sim.simulation_stopped
    pause(0.1);
end

% Restore the original idle loop frequency:
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps);

% Close CoppeliaSim scene
sim.closeScene();

figure(1)
subplot(2,1,1)
plot(t,xp,t,yp,t,fp,'LineWidth',2),grid
legend('$x_{p}(t)$','$y_{p}(t)$','$\phi_{p}(t)$','Interpreter','latex')
xlabel('t [s]')
subplot(2,1,2)
plot(t,up,t,om,t,vr,t,vl,'LineWidth',2),grid
legend('$\upsilon(t)$','$\omega(t)$','$\dot{\varphi}_{R}(t)$','$\dot{\varphi}_{L}(t)$','Interpreter','latex')
xlabel('t [s]')
%
figure(2)
axis equal
hold on
plot(xp,yp,'k:','LineWidth',2.0),grid
xlabel('$x_{p}(t), m$','Interpreter','latex'),ylabel('$y_{p}(t), m$','Interpreter','latex')
for i=1:round(length(xp)/20):length(xp)
    drawRobot(xp(i),yp(i),fp(i),0.02);
end
hold off

fprintf('Program ended\n');
