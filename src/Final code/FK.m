function [position] = FK(theta1,theta2,theta3,theta4)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
l1 = 17.55; % height (ground to second joint)
l2 = 20; % length of first limb
l3 = 20; % length of second limb
l4 = 16; % length of grabber

a0 = 0;
a1 = 0;
a2=l2;
a3=l3;

d1=l1;
d2 = 0;
d3 = 0;
d4 = 0;

alpha0 = 0;
alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;

Rx_10 = [1 0 0 0;
        0 cos(alpha0) -sin(alpha0) 0;
        0 sin(alpha0) cos(alpha0) 0;
        0 0 0 1];

Dx_10 = [1 0 0 a0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_10 = [cos(theta1) -sin(theta1) 0 0;
        sin(theta1) cos(theta1) 0 0;
        0 0 1 0
        0 0 0 1];

Dz_10 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d1;
        0 0 0 1];

Rx_21 = [1 0 0 0;
        0 cos(alpha1) -sin(alpha1) 0;
        0 sin(alpha1) cos(alpha1) 0;
        0 0 0 1];
    
Dx_21 = [1 0 0 a1;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_21 = [cos(theta2) -sin(theta2) 0 0;
        sin(theta2) cos(theta2) 0 0;
        0 0 1 0
        0 0 0 1];

Dz_21 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d2;
        0 0 0 1];

Rx_32 = [1 0 0 0;
        0 cos(alpha2) -sin(alpha2) 0;
        0 sin(alpha2) cos(alpha2) 0;
        0 0 0 1];

Dx_32 = [1 0 0 a2;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_32 = [cos(theta3) -sin(theta3) 0 0;
        sin(theta3) cos(theta3) 0 0;
        0 0 1 0
        0 0 0 1];

Dz_32 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d3;
        0 0 0 1];

Rx_43 = [1 0 0 0;
        0 cos(alpha3) -sin(alpha3) 0;
        0 sin(alpha3) cos(alpha3) 0;
        0 0 0 1];

Dx_43 = [1 0 0 a3;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_43 = [cos(theta4) -sin(theta4) 0 0;
        sin(theta4) cos(theta4) 0 0;
        0 0 1 0
        0 0 0 1];

Dz_43 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d4;
        0 0 0 1];
    



T_10 = Rx_10 * Dx_10 * Rz_10 * Dz_10;
T_21 = Rx_21 * Dx_21 * Rz_21 * Dz_21;
T_32 = Rx_32 * Dx_32 * Rz_32 * Dz_32;
T_43 = Rx_43 * Dx_43 * Rz_43 * Dz_43;

T_40 = T_10 * T_21 * T_32 * T_43;
original = [0;0;0;1];
step5 = T_10 * T_21 * T_32 * T_43 * original;

x = step5(1);
y = step5(2);
z = step5(3) - l4;
position = [x,y,z];
end