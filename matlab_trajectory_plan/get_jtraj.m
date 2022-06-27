function [valid,q,qd,qdd] =  get_jtraj(start_joints,via_points,step)

 
    %base_xyz=[0,0,0];
    %robot=puma560_robot(start_angles,base_xyz,0);
    
    current_joints=start_joints;
    
    % ≥ı ºªØ
    [q,qd,qdd] =  init_q_qd_qdd(current_joints);

    for i =2:size(via_points,2)
        cartesian_position_DH=cell2mat(via_points(i));
        [valid,result_list] = inverse_kinematics(cartesian_position_DH);
        if valid 
            best_joints = find_best_joints_with_min_change(current_joints,result_list);

            [q,qd,qdd] =  add_jtraj(current_joints,best_joints,q,qd,qdd,step);
            current_joints=best_joints;
        else
           disp('get_jtraj: can not reach point:');
           i
           break;
        end

    end
end
    
    
function [q,qd,qdd] =  add_jtraj(First_Theta,Final_Theta,q,qd,qdd,step)

    [q1,qd1,qdd1] = jtraj(First_Theta,Final_Theta,step);
    for i =2:size(q1,1)
        q=[q;q1(i,:)];
        qd=[qd;qd1(i,:)];
        qdd=[qdd;qdd1(i,:)];
    end
end


function [q,qd,qdd] =  init_q_qd_qdd(init_theta)
    q = init_theta;
    qd = [0 0 0 0 0 0];
    qdd = [0 0 0 0 0 0];
end
    

