function [valid,joint_variables]=inverse_kinematics_one(cartesian_DH,current_joint_variables)

     [valid,result_list] = inverse_kinematics(cartesian_DH);
     if valid
        joint_variables = find_best_joints_with_min_change(current_joint_variables,result_list);
     else
         joint_variables=[];
     end
end