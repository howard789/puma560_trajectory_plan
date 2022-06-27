function [valid,joint_q,joint_qd,joint_qdd] = get_joint_data(start_joints,q,tstep)

    valid=1;
    
    joint_q = zeros(size(q,3),6);
    joint_qd = zeros(size(q,3),6);
    joint_qdd = zeros(size(q,3),6);
    
    current_joints = start_joints;
    for i =1:size(q,3)
        disp('---------------------------------------------------------------')
        if i==94
            1;
        end
        
       [ik_valid,results] = inverse_kinematics(q(:,:,i));
       if ik_valid 
           [best_joints,min_change_degs] = find_best_joints_with_min_change(current_joints,results);
           if min_change_degs>1
            current_joints
            best_joints
            sprintf('i: %.0f distance %.2f',i,norm(best_joints-current_joints))
            %error('min_change_degs too big');
           end
           
           
           
           
           
           joint_q(i,:)=best_joints;
           current_joints=best_joints;
       else
            valid=0;
            sprintf('fail to get ik points: %.0f',i)
       end
       
      


    end
    
    if valid 
        for i =2:size(joint_q,1)
            joint_qd(i,:)=(joint_q(i,:)-joint_q(i-1,:))/tstep;
        end
    end
    
    if valid 
        for i =1:size(joint_qd,1)
            if i==size(joint_qd,1)
                 joint_qdd(i,:)=-joint_qd(i,:);
            else
                joint_qdd(i,:)= joint_qd(i+1,:)-joint_qd(i,:);
            end
        end
    end
    
    

end