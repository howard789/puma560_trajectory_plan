function puma560_display_joints(robot,joint_rads)
    %robot.display() 
    % Pose Kinematics
    %theta = [0, 0, 0, 0, 0, 0]*pi/180;
    %robot.plot(joint_rads); 
    T=robot.fkine(joint_rads);    %Ä©¶ËÖ´ÐÐÆ÷Î»×Ë
    %disp('fkine:');
    %disp(T);
    %robot.teach(joint_rads);
    %robot.display();    
    robot.plot(joint_rads);
end
    