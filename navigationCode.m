%  r=radius of wheel, l=distance from robot center to wheel hub (meters)
r=0.05, l=0.2792

% alpha angles
a1=32.5, a2=147.5, a3=-147.5, a4=-32.5

% beta angles
b1=57.5, b2=-57.5, b3=-122.5, b4=122.5

% gamma angles
g1=45, g2=135, g3=45, g4=135

% k=alpha+beta+gamma
k1=a1+b1+g1, k2=a2+b2+g2, k3=a3+b3+g3, k4=a4+b4+g4

% A and B matrix for wheel equations
A = [sind(k1), -cosd(k1), -l*cosd(b1+g1) ; sind(k2), -cosd(k2), -l*cosd(b2+g2) ; sind(k3), -cosd(k3), -l*cosd(b3+g3) ; sind(k4), -cosd(k4), -l*cosd(b4+g4)]
B = [r*cosd(g1),0,0,0;0,r*cosd(g2),0,0;0,0,r*cosd(g3),0;0,0,0,r*cosd(g4)]

% Vx,Vy,thetadot are robot velocities; w1,w2,w3,w4 are joint velocities
Vx=0; Vy=0; thetadot=0; w1=0; w2=0; w3=0; w4=0; 

% Robot velocities as inputs
Vx=0.05;
Vy=0.05;
thetadot=0;

sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

 if (clientID>-1)
        disp('Connected to remote API server');
      
h=[0 0 0 0];
s=[0 0];

%handle objects
[returnCode,s(1)]=sim.simxGetObjectHandle(clientID,'Proximity_front',sim.simx_opmode_blocking);
[returnCode,s(2)]=sim.simxGetObjectHandle(clientID,'Proximity_rear',sim.simx_opmode_blocking);

[returnCode,b]=sim.simxGetObjectHandle(clientID,'youBot',sim.simx_opmode_blocking);

[returnCode,h(1)]=sim.simxGetObjectHandle(clientID,'rollingJoint_rr',sim.simx_opmode_blocking);
[returnCode,h(2)]=sim.simxGetObjectHandle(clientID,'rollingJoint_fr',sim.simx_opmode_blocking);
[returnCode,h(3)]=sim.simxGetObjectHandle(clientID,'rollingJoint_rl',sim.simx_opmode_blocking);
[returnCode,h(4)]=sim.simxGetObjectHandle(clientID,'rollingJoint_fl',sim.simx_opmode_blocking);

%initial velocity vector
Vx=0.1*vely_1;
Vy=0.1*velx_1;

%deadzone
d=0.05;
    
while true
%get robot position from CoppeliaSim
[returnCode,positionRobot]=sim.simxGetObjectPosition(clientID,b,-1,sim.simx_opmode_streaming);

robotX = positionRobot(1);
robotY = positionRobot(2);

robotXrounded = round(robotX,2);
robotYrounded = round(robotY,2);

% V matrix is robot velocity, W matrix is wheel velocity
V = [Vx;Vy;thetadot];
W = [w1;w2;w3;w4];

% A*V=B*W is general matrix equation

Y = inv(B); % Y is inverse of B matrix

W=Y*A*V; % calculate w1;w2;w3;w4 

%check waypoint number and update velocity vector
if waypointNumber == 5
    if (robotXrounded < x2+d) && (robotXrounded > x2-d) && (robotYrounded < y2+d) && (robotYrounded > y2-d)
        Vx=0.07*vely_2;
        Vy=0.07*velx_2;
    elseif (robotXrounded < x3+d) && (robotXrounded > x3-d) && (robotYrounded < y3+d) && (robotYrounded > y3-d)
        Vx=0.07*vely_3;
        Vy=0.07*velx_3;
    elseif (robotXrounded < x4+d) && (robotXrounded > x4-d) && (robotYrounded < y4+d) && (robotYrounded > y4-d)
        Vx=0.07*vely_4;
        Vy=0.07*velx_4;
    elseif (robotXrounded < x5+d) && (robotXrounded > x5-d) && (robotYrounded < y5+d) && (robotYrounded > y5-d)
        Vx=0;
        Vy=0;
    end
end

if waypointNumber == 6
    if (robotXrounded < x2+d) && (robotXrounded > x2-d) && (robotYrounded < y2+d) && (robotYrounded > y2-d)
        Vx=0.07*vely_2;
        Vy=0.07*velx_2;
    elseif (robotXrounded < x3+d) && (robotXrounded > x3-d) && (robotYrounded < y3+d) && (robotYrounded > y3-d)
        Vx=0.07*vely_3;
        Vy=0.07*velx_3;
    elseif (robotXrounded < x4+d) && (robotXrounded > x4-d) && (robotYrounded < y4+d) && (robotYrounded > y4-d)
        Vx=0.07*vely_4;
        Vy=0.07*velx_4;
    elseif (robotXrounded < x5+d) && (robotXrounded > x5-d) && (robotYrounded < y5+d) && (robotYrounded > y5-d)
        Vx=0.07*vely_5;
        Vy=0.07*velx_5;
    elseif (robotXrounded < x6+d) && (robotXrounded > x6-d) && (robotYrounded < y6+d) && (robotYrounded > y6-d)
        Vx=0;
        Vy=0;
    end
end


if waypointNumber == 7
    if (robotXrounded < x2+d) && (robotXrounded > x2-d) && (robotYrounded < y2+d) && (robotYrounded > y2-d)
        Vx=0.07*vely_2;
        Vy=0.07*velx_2;
    elseif (robotXrounded < x3+d) && (robotXrounded > x3-d) && (robotYrounded < y3+d) && (robotYrounded > y3-d)
        Vx=0.07*vely_3;
        Vy=0.07*velx_3;
    elseif (robotXrounded < x4+d) && (robotXrounded > x4-d) && (robotYrounded < y4+d) && (robotYrounded > y4-d)
        Vx=0.07*vely_4;
        Vy=0.07*velx_4;
    elseif (robotXrounded < x5+d) && (robotXrounded > x5-d) && (robotYrounded < y5+d) && (robotYrounded > y5-d)
        Vx=0.07*vely_5;
        Vy=0.07*velx_5;
    elseif (robotXrounded < x6+d) && (robotXrounded > x6-d) && (robotYrounded < y6+d) && (robotYrounded > y6-d)
        Vx=0.07*vely_6;
        Vy=0.07*velx_6;
    elseif (robotXrounded < x7+d) && (robotXrounded > x7-d) && (robotYrounded < y7+d) && (robotYrounded > y7-d)
        Vx=0;
        Vy=0;
    end
end

if waypointNumber == 8
    if (robotXrounded < x2+d) && (robotXrounded > x2-d) && (robotYrounded < y2+d) && (robotYrounded > y2-d)
        Vx=0.07*vely_2;
        Vy=0.07*velx_2;
    elseif (robotXrounded < x3+d) && (robotXrounded > x3-d) && (robotYrounded < y3+d) && (robotYrounded > y3-d)
        Vx=0.07*vely_3;
        Vy=0.07*velx_3;
    elseif (robotXrounded < x4+d) && (robotXrounded > x4-d) && (robotYrounded < y4+d) && (robotYrounded > y4-d)
        Vx=0.07*vely_4;
        Vy=0.07*velx_4;
    elseif (robotXrounded < x5+d) && (robotXrounded > x5-d) && (robotYrounded < y5+d) && (robotYrounded > y5-d)
        Vx=0.07*vely_5;
        Vy=0.07*velx_5;
    elseif (robotXrounded < x6+d) && (robotXrounded > x6-d) && (robotYrounded < y6+d) && (robotYrounded > y6-d)
        Vx=0.07*vely_6;
        Vy=0.07*velx_6;
    elseif (robotXrounded < x7+d) && (robotXrounded > x7-d) && (robotYrounded < y7+d) && (robotYrounded > y7-d)
        Vx=0.07*vely_7;
        Vy=0.07*velx_7;
    elseif (robotXrounded < x8+d) && (robotXrounded > x8-d) && (robotYrounded < y8+d) && (robotYrounded > y8-d)
        Vx=0;
        Vy=0;
    end
end

%send wheel velocities to robot
sim.simxSetJointTargetVelocity(clientID,h(1),W(3),sim.simx_opmode_oneshot);
sim.simxSetJointTargetVelocity(clientID,h(2),W(4),sim.simx_opmode_oneshot);
sim.simxSetJointTargetVelocity(clientID,h(3),W(2),sim.simx_opmode_oneshot);
sim.simxSetJointTargetVelocity(clientID,h(4),W(1),sim.simx_opmode_oneshot);

end


 else
        disp('Failed connecting to remote API server');
 end
    sim.delete(); % call the destructor!
    
    disp('Program ended');