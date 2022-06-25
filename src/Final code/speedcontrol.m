function [angles_detail] = speedcontrol(track_position)

    angles = [];
    for i = 1:length(track_position(:,1))  
        [h_theta1, h_theta2, h_theta3, h_theta4] = inv_K2(track_position(i,1), track_position(i,2), track_position(i,3));
        
        offset1 = -0.32;
        if track_position(i,2) > 1.49
            offset1 = -0.325;
        end
        
        offset2 = - pi/2;
        offset3 = pi/2 -0.02;
        offset4 = 0.43;

%         if track_position(i,1) < 16.5 && track_position(i,2) > 16.5
%             offset1 = -0.45;
%             offset2 = - pi/2 +0.01;
%         end
        
        h_theta1_motor = h_theta1 + offset1;
        h_theta2_motor = h_theta2 + offset2;
        h_theta2_increase = h_theta2_motor;
        h_theta3_motor = -(h_theta3 + h_theta2_increase+ offset3);
        h_theta3_increase = h_theta3_motor;
        h_theta4_motor = -h_theta3_increase + offset4;
   
        angles(i,:) = [h_theta1_motor, h_theta2_motor, h_theta3_motor, h_theta4_motor];
    end
    
    angles_detail = [];
    for i = 1:length(angles(:,1))-1
        step_number1 = int8(abs(angles(i,1)- angles(i+1,1))/0.01);
        step_number2 = int8(abs(angles(i,2)- angles(i+1,2))/0.01);
        step_number3 = int8(abs(angles(i,3)- angles(i+1,3))/0.01);
        step_number4 = int8(abs(angles(i,4)- angles(i+1,4))/0.01);
        step_number = max([step_number1,step_number2,step_number3,step_number4]);

%         step_number = 20;
        if angles(i,1) ~= angles(i+1,1) || angles(i,2) ~= angles(i+1,2) || angles(i,3) ~= angles(i+1,3) || angles(i,4) ~= angles(i+1,4)
            angles_detail = [angles_detail;[linspace(angles(i,1),angles(i+1,1),step_number)',...
                             linspace(angles(i,2),angles(i+1,2),step_number)',...
                             linspace(angles(i,3),angles(i+1,3),step_number)',...
                             linspace(angles(i,4),angles(i+1,4),step_number)']];
        else 
            angles_detail = [angles_detail;angles(i,:)];
        end
    end
end