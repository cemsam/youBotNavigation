clear all
clc
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
syms Vx, syms Vy, syms thetadot, syms w1, syms w2, syms w3, syms w4

% Wheel velocities as inputs
w1=input('Enter 1st wheel velocity=');
w2=input('Enter 2nd wheel velocity=');
w3=input('Enter 3rd wheel velocity=');
w4=input('Enter 4th wheel velocity=');

% V matrix is robot velocity, W matrix is wheel velocity
V = [Vx;Vy;thetadot];
W = [w1;w2;w3;w4];

% A*V=B*W is general matrix equation

Y = pinv(A); % Y is inverse of A matrix

V=Y*B*W % calculate Vx;Vy;thetadot 


