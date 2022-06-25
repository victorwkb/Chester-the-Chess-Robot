function  [angle]= getCurrentAngle(t1,t2,t3,t4)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
        offset1 = -0.32;
        offset2 = - pi/2;
        offset3 = pi/2 -0.02;
        offset4 = 0.43;

        
        h_theta1 = t1 - offset1;
        h_theta2 = t2 - offset2;
        h_theta2_increase = t2;
        h_theta3 = -t3 - h_theta2_increase - offset3;
        h_theta3_increase = t3;
        h_theta4 = -h_theta3_increase + offset4;
        angle = [h_theta1, h_theta2, h_theta3, h_theta4];
end