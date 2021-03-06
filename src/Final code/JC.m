function [J] = JC(theta1,theta2,theta3,theta4)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
l1 = 17.55; % height (ground to second joint)
l2 = 20; % length of first limb
l3 = 20; % length of second limb

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

Rx_10 = [1 0 0;
        0 cos(alpha0) -sin(alpha0);
        0 sin(alpha0) cos(alpha0);
        ];

Rz_10 = [cos(theta1) -sin(theta1) 0;
        sin(theta1) cos(theta1) 0;
        0 0 1
        ];

Rx_21 = [1 0 0;
        0 cos(alpha1) -sin(alpha1);
        0 sin(alpha1) cos(alpha1);
        ];


Rz_21 = [cos(theta2) -sin(theta2) 0;
        sin(theta2) cos(theta2) 0;
        0 0 1];


Rx_32 = [1 0 0;
        0 cos(alpha2) -sin(alpha2);
        0 sin(alpha2) cos(alpha2);
        ];

Rz_32 = [cos(theta3) -sin(theta3) 0;
        sin(theta3) cos(theta3) 0;
        0 0 1];

Rx_43 = [1 0 0;
        0 cos(alpha3) -sin(alpha3);
        0 sin(alpha3) cos(alpha3);
        ];

Rz_43 = [cos(theta4) -sin(theta4) 0;
        sin(theta4) cos(theta4) 0;
        0 0 1];

T_10 = Rx_10* Rz_10;
T_21 = Rx_21* Rz_21;
T_32 = Rx_32* Rz_32;
T_43 = Rx_43* Rz_43;

P1c1_1=[0;0;l1]; 
P2c2_2=[l2;0;0]; 
P3c3_3=[l3;0;0];
%P4c4_4=[lc4;0;0];

P2c2_0= T_10*T_21*P2c2_2;
P1c2_0= T_10 * P1c1_1+P2c2_0;
P3c3_0= T_10*T_21*T_32*P3c3_3; 
P2c3_0= P2c2_0+P3c3_0;
P1c3_0= T_10 * P1c1_1 + P2c2_0 + P3c3_0;

Z_1 = T_10*[0 0 1]';
Z_2 = T_10*T_21 * Z_1;
Z_3 = T_10*T_21 * T_32 * Z_1;

J=[cross(Z_1,P1c3_0) cross(Z_2,P2c3_0) cross(Z_3,P3c3_0)];

end