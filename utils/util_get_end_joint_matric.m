function T= util_get_end_joint_matric(alpha,beta,gamma,x,y,z,end2targetT)

    T_target = util_get_target_matric(alpha,beta,gamma,x,y,z);
    T = T_target*inv(end2targetT);

end
