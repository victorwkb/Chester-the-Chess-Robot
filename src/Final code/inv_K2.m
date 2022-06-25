function [theta_1,theta_2,theta_3_final,theta_4] = inv_K2(x,y,z)
%inv_K Calculates inverse kinematics of joint angles
%   x,y,z - Coordinates of end effector in task space

%   [theta_1,theta_2,theta_3_final,theta_4] - Vector of motor angles

%% Arm specifications
l1 = 17.55; % height (ground to second joint)
l2 = 20; % length of first limb
l3 = 20; % length of second limb
l4 = 16; % length of grabber
if x > 16.5 && x <= 22.5
    l4 = 16.4;
elseif x > 22.5
    l4 = 16.9;
end

z = z + l4;

%% Inverse Kinematics
syms theta_3 theta_5 theta_6

BD = x^2 + y^2 +((z) - l1)^2 == l2^2 + l3^2 - 2*l2*l3*cos(pi-theta_3);

theta_3 = solve(BD);

theta_3_final = theta_3(find(theta_3 > 0));
theta_3_final = -theta_3_final(find(theta_3_final < pi));
bd = sqrt(x^2 + y^2 +((z) - l1)^2);
theta_5 = solve(l3^2 == bd^2 +l2^2 - 2*bd*l2*cos(theta_5));
theta_5 = theta_5(find(theta_5>=0));

theta_6 = solve(x^2 + y^2 + (z)^2== bd^2 + l1^2 - 2*bd*l1*cos(theta_6));
if z > l1 
    theta_6 = theta_6(find(theta_6 < pi));
else 
    theta_6 = theta_6(find(theta_6 >= 0));
end

theta_2 = pi/2 - (pi - (theta_5 + theta_6));
theta_1 = atan(y/x);
theta_4 = pi - (pi/2 - theta_2) + theta_3_final;

theta_2 = eval(theta_2);
theta_3_final = eval(theta_3_final);
theta_4 = eval(theta_4);
% %% Mechanics 
% a = sqrt(x^2+y^2);
% m = -1/a*(-l1+z+l4);
% n = (l2^2-l3^2+(l4)^2-(z-l1)^2-a^2)/2/a;
% p = 2*(m*n-l4)/(m^2+1);
% q = ((n)^2+l4^2-l3^2)/(m^2+1);
% r1=(-p+sqrt(p^2-4*q))/2;
% %r2=(-p-sqrt(p^2-4*q))/2
% s1=m*r1+n;
% %s2=m*r2+n
% Theta2_1=atan((a+s1)/(r1-l1+z));
% %if Theta2_1<0 
%   %  Theta2_1=Theta2_1+180;
% %end
% %Theta2_2=atand((a+s2)/(r2-l1+z))
% Theta4_1=atan(-s1/(r1-l4));
% 
% 
% %Theta_4_2=atand(-s2/(r2-l4))
% Theta3_1=pi-Theta4_1-Theta2_1;
% %if Theta3_1<0 Theta4_1=Theta4_1-180;Theta3_1=180-Theta4_1-Theta2_1;
% %end
% %Theta3_2=180-Theta4_2-Theta2_2
% theta1=atan(y/x);
% theta2=Theta2_1;
% theta3=Theta3_1;
% theta4=Theta4_1;
end

