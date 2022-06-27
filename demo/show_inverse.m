function show_inverse(T)
    % end joint 
    current_joint_variables=[0 0 0 0 0 0];
    %cartesian_position_DH =[0 0 0 x;0 0 0 y;0 0 0 z;0 0 0 1];
     [valid,joint_variables]=inverse_kinematics_one(T,current_joint_variables);

    if valid 
        base_xyz =[0 0 0];
        robot = puma560_robot(base_xyz);
        robot.display();
        robot.plot(joint_variables);
        
        
    else
        disp('can not reach')
    end
end