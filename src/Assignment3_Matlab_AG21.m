clear all
close all
clc
%% Trajectory Generation

initial = [22 14 0];
final = [22 -6 0];
initial_dot = [0 0 0];
final_dot = [0 0 0];
xi = initial(1);
yi = initial(2);
zi = initial(3);
xi_dot = initial_dot(1);
yi_dot = initial_dot(2);
zi_dot = initial_dot(3);
xf = final(1);
yf = final(2);
zf = final(3);
xf_dot = final_dot(1);
yf_dot = final_dot(2);
zf_dot = final_dot(3);

x2 = xi;
y2 = yi;
z2 = zi;
if z2 > 7
    z2 = zi;
else
    z2 = 7;
end
x3 = xf;
y3 = yf;
z3 = zf;
if z3 > 7
    z3 = zf;
else
    z3 = 7;
end


speed = 0;
x2_dot = 0;
y2_dot = speed;
z2_dot = 0;
x3_dot = speed;
y3_dot = speed;
z3_dot = 0;

 

syms ax_0 ax_1 ax_2 ax_3 ay_0 ay_1 ay_2 ay_3 az_0 az_1 az_2 az_3
syms bx_0 bx_1 bx_2 bx_3 by_0 by_1 by_2 by_3 bz_0 bz_1 bz_2 bz_3
syms cx_0 cx_1 cx_2 cx_3 cy_0 cy_1 cy_2 cy_3 cz_0 cz_1 cz_2 cz_3
t1 = 1;
t2 = 2;

matrix1 = [1 0 0 0;
 1 t1 t1^2 t1^3;
 0 1 0 0;
 0 1 2*t1 3*t1^2];

matrix2 = [1 0 0 0;
 1 t2 t2^2 t2^3;
 0 1 0 0;
 0 1 2*t2 3*t2^2];

equation_x = matrix1 * [ax_0; ax_1; ax_2; ax_3] == [xi; x2; xi_dot; x2_dot];
equation_y = matrix1 * [ay_0; ay_1; ay_2; ay_3] == [yi; y2; yi_dot; y2_dot];
equation_z = matrix1 * [az_0; az_1; az_2; az_3] == [zi; z2; zi_dot; z2_dot];

equation2_x = matrix2 * [bx_0; bx_1; bx_2; bx_3] == [x2; x3; x2_dot; x3_dot];
equation2_y = matrix2 * [by_0; by_1; by_2; by_3] == [y2; y3; y2_dot; y3_dot];
equation2_z = matrix2 * [bz_0; bz_1; bz_2; bz_3] == [z2; z3; z2_dot; z3_dot];

equation3_x = matrix1 * [cx_0; cx_1; cx_2; cx_3] == [x3; xf; x3_dot; xf_dot];
equation3_y = matrix1 * [cy_0; cy_1; cy_2; cy_3] == [y3; yf; y3_dot; yf_dot];
equation3_z = matrix1 * [cz_0; cz_1; cz_2; cz_3] == [z3; zf; z3_dot; zf_dot];

ax = solve(equation_x,[ax_0 ax_1 ax_2 ax_3]);
ay = solve(equation_y,[ay_0 ay_1 ay_2 ay_3]);
az = solve(equation_z,[az_0 az_1 az_2 az_3]);
ax = [ax.ax_0 ax.ax_1 ax.ax_2 ax.ax_3]
ay = [ay.ay_0 ay.ay_1 ay.ay_2 ay.ay_3]
az = [az.az_0 az.az_1 az.az_2 az.az_3]

bx = solve(equation2_x,[bx_0 bx_1 bx_2 bx_3]);
by = solve(equation2_y,[by_0 by_1 by_2 by_3]);
bz = solve(equation2_z,[bz_0 bz_1 bz_2 bz_3]);
bx = [bx.bx_0 bx.bx_1 bx.bx_2 bx.bx_3]
by = [by.by_0 by.by_1 by.by_2 by.by_3]
bz = [bz.bz_0 bz.bz_1 bz.bz_2 bz.bz_3]

cx = solve(equation3_x,[cx_0 cx_1 cx_2 cx_3]);
cy = solve(equation3_y,[cy_0 cy_1 cy_2 cy_3]);
cz = solve(equation3_z,[cz_0 cz_1 cz_2 cz_3]);
cx = [cx.cx_0 cx.cx_1 cx.cx_2 cx.cx_3]
cy = [cy.cy_0 cy.cy_1 cy.cy_2 cy.cy_3]
cz = [cz.cz_0 cz.cz_1 cz.cz_2 cz.cz_3]

t1_period = 0:0.01:1;
t2_period = 0:0.01:2;
t3_period = 0:0.01:1;
for i = 1:length(t1_period)
    x_track(i) = ax * [1; t1_period(i); t1_period(i)^2; t1_period(i)^3];
    y_track(i) = ay * [1; t1_period(i); t1_period(i)^2; t1_period(i)^3];
    z_track(i) = az * [1; t1_period(i); t1_period(i)^2; t1_period(i)^3];
end
for i = length(t1_period)+1:length(t1_period) + length(t2_period)
    x_track(i) = bx * [1; t2_period(i-length(t1_period)); t2_period(i-length(t1_period))^2; t2_period(i-length(t1_period))^3];
    y_track(i) = by * [1; t2_period(i-length(t1_period)); t2_period(i-length(t1_period))^2; t2_period(i-length(t1_period))^3];
    z_track(i) = bz * [1; t2_period(i-length(t1_period)); t2_period(i-length(t1_period))^2; t2_period(i-length(t1_period))^3];
end
for i = length(t1_period) + length(t2_period) + 1:length(t1_period) + length(t2_period) + length(t3_period)
    x_track(i) = cx * [1; t3_period(i-length(t1_period) - length(t2_period)); t3_period(i-length(t1_period) - length(t2_period))^2; t3_period(i-length(t1_period) - length(t2_period))^3];
    y_track(i) = cy * [1; t3_period(i-length(t1_period) - length(t2_period)); t3_period(i-length(t1_period) - length(t2_period))^2; t3_period(i-length(t1_period) - length(t2_period))^3];
    z_track(i) = cz * [1; t3_period(i-length(t1_period) - length(t2_period)); t3_period(i-length(t1_period) - length(t2_period))^2; t3_period(i-length(t1_period) - length(t2_period))^3];
end

figure(1)
plot3(x_track,y_track,z_track)
hold on 
scatter3(x_track(end),y_track(end),z_track(end),'red','filled')
scatter3(x_track(1),y_track(1),z_track(1),'green','filled')
scatter3(22,14,7,'blue','filled')
scatter3(22,-6,7,'blue','filled')
title("XYZ endeffector plot")
xlabel('X')
ylabel('Y')
zlabel('Z')

figure(3)
plot(y_track,x_track)
title("XY endeffector plot")
xlabel('Y')
ylabel('x')

figure(4)
plot(y_track,z_track)
hold on 
plot(-6,[0:1:6],"rx")
plot(14,[0:1:6],"rx")
plot(3,[7:1:13],"rx")
plot(3,[0:1:6],"gx")
title("XZ endeffector plot")
xlabel('Y')
ylabel('Z')
xlim([-7 15])
ylim([0 15])

%% Q4


T_ID = [1 41 81 121 161 201 241 281 321 361 401];

for i = 1:11

x=x_track(T_ID(i));
y=y_track(T_ID(i));
z=z_track(T_ID(i));

[q1 q2 q3 q4] = inv_K(x,y,z);

l1 = 15; % height (ground to second joint)
l2 = 27; % length of first arm
l3 = 23;% length of third arm
l4 = 10;% length of forth arm
a0 = 0;
a1 = 0;
a2=l2;
a3=l3;
aE=l4;
d1=l1;
d2 = 0;
d3 = 0;
d4 = 0;
dE = 0;
alpha0 = 0;
alpha1 = 90;
alpha2 = 0;
alpha3 = 0;
alphaE = 0;
theta1 = q1;
theta2 = 90-q2;
theta3 = -q3;
theta4 = -q4;
thetaE = 0;
Rx_10 = [1 0 0 0;
        0 cosd(alpha0) -sind(alpha0) 0;
        0 sind(alpha0) cosd(alpha0) 0;
        0 0 0 1];

Dx_10 = [1 0 0 a0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_10 = [cosd(theta1) -sind(theta1) 0 0;
        sind(theta1) cosd(theta1) 0 0;
        0 0 1 0;
        0 0 0 1];

Dz_10 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d1;
        0 0 0 1];

Rx_21 = [1 0 0 0;
        0 cosd(alpha1) -sind(alpha1) 0;
        0 sind(alpha1) cosd(alpha1) 0;
        0 0 0 1];
Dx_21 = [1 0 0 a1;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_21 = [cosd(theta2) -sind(theta2) 0 0;
        sind(theta2) cosd(theta2) 0 0;
        0 0 1 0;
        0 0 0 1];

Dz_21 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d2;
        0 0 0 1];

Rx_32 = [1 0 0 0;
        0 cosd(alpha2) -sind(alpha2) 0;
        0 sind(alpha2) cosd(alpha2) 0;
        0 0 0 1];

Dx_32 = [1 0 0 a2;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_32 = [cosd(theta3) -sind(theta3) 0 0;
        sind(theta3) cosd(theta3) 0 0;
        0 0 1 0;
        0 0 0 1];

Dz_32 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d3;
        0 0 0 1];

Rx_43 = [1 0 0 0;
        0 cosd(alpha3) -sind(alpha3) 0;
        0 sind(alpha3) cosd(alpha3) 0;
        0 0 0 1];

Dx_43 = [1 0 0 a3;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_43 = [cosd(theta4) -sind(theta4) 0 0;
        sind(theta4) cosd(theta4) 0 0;
        0 0 1 0;
        0 0 0 1];

Dz_43 = [1 0 0 0;
        0 1 0 0;
        0 0 1 d4;
        0 0 0 1];

Rx_E4 = [1 0 0 0;
        0 cosd(alphaE) -sind(alphaE) 0;
        0 sind(alphaE) cosd(alphaE) 0;
        0 0 0 1];

Dx_E4 = [1 0 0 aE;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Rz_E4 = [cosd(thetaE) -sind(thetaE) 0 0;
        sind(thetaE) cosd(thetaE) 0 0;
        0 0 1 0;
        0 0 0 1];

Dz_E4 = [1 0 0 0;
        0 1 0 0;
        0 0 1 dE;
        0 0 0 1];

T_10 = Dx_10 * Rx_10 * Dz_10 * Rz_10;
T_21 = Dx_21 * Rx_21 * Dz_21 * Rz_21;
T_32 = Dx_32 * Rx_32 * Dz_32 * Rz_32;
T_43 = Dx_43 * Rx_43 * Dz_43 * Rz_43;
T_E4 = Dx_E4 * Rx_E4 * Dz_E4 * Rz_E4;

T_E0 = T_10 * T_21 * T_32 * T_43 * T_E4;
original = [0;0;0;1];
step1 = original;
step2 = T_10*original;
step3 = T_10*T_21*original;
step4 = T_10 * T_21 * T_32 * original;
step5 = T_10 * T_21 * T_32 * T_43 * original;
step6 = T_E0* original;
graph = [step1,step2,step3,step4,step5,step6];


figure(5)
plot(graph(2,:),graph(3,:),'g','LineWidth',1.5)
hold on 
scatter(graph(2,:),graph(3,:),'red','filled')
xlabel('Y')
ylabel('Z')

end