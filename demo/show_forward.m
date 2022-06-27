function show_forward(joints)
    % end joint 
    
    joint_rads=joints/180*pi;
    T = forward_kinematics(6,joint_rads)

    base_xyz =[0 0 0];
    robot = puma560_robot(base_xyz);
    robot.display();    
    robot.plot(joint_rads);
    
end