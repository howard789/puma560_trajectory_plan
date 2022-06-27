function [valid,q,qd,qdd] = get_joints_trajectory_plan(cartesian_DH,target_velocity,tstep,tacc)

    if size(cartesian_DH,3)<3
        error('There should be at least 3 points')
    end
    
    valid=1;

    current_joint_variables=[0 0 0 0 0 0];
    % prepare schedule
    [time_schedule,joint_velocity_schedule,joint_acc_schedule,joint_schedule] = get_schedule(cartesian_DH,target_velocity,current_joint_variables,tacc);
    

    q=joint_schedule(1,:);
    qd=zeros(1,6);
    qdd=zeros(1,6);
    
    points_num_record=[1];
    
    for i = 0:size(time_schedule,2)-1
      
       % 求出一个片段的q qd qdd
        schedule_start_col=i;
        schedule_end_col=i+1;
        [q1,qd1,qdd1]=get_trajectory(schedule_start_col,schedule_end_col,time_schedule,cartesian_DH,tstep,joint_schedule,joint_velocity_schedule,joint_acc_schedule);
       
        
       points_num_record=[points_num_record;points_num_record(end,1)+size(q1,1)];

        
        q=cat(1,q,q1);
        qd=cat(1,qd,qd1);
        qdd=cat(1,qdd,qdd1);
    
        
    end
    points_num_record
end




function [q,qd,qdd]=get_trajectory(schedule_start_col,schedule_end_col,time_schedule,DH_points,tstep, ...
        joint_schedule,joint_velocity_schedule,joint_acc_schedule)

   row_distance=1;
   row_time=2;
   row_tacc=3;


    if schedule_start_col==0
        %从第一个点到第1和2的直线段,当schedule_start_col=1时是从1和2的直线段到第2个点曲线段结束
         tk=time_schedule(row_tacc,1);
         
         end_time=0.5*tk;
         points_num=(0.5*tk)/tstep;
         if mod(points_num,1)==0
            points_num=floor(points_num);
        else
            points_num=floor(points_num)+1;
         end
        
        q=zeros(points_num,6);
        qd=zeros(points_num,6);
        qdd=zeros(points_num,6);
        
        qdd_k = joint_acc_schedule(1,:);

        t=0;
        for i =1:points_num
  
          
            if i==points_num
                t=end_time;
            else
                t=t + tstep;
            end
          
          
          
          q(i,:)=joint_schedule(1,:)+0.5*qdd_k*(t+0.5*tk);
          qd(i,:)=qdd_k*(t+0.5*tk);
          qdd(i,:)=qdd_k; 
          
         %sprintf('From point %.0f to point %.0f  parabolic subpoint %.0f  of %.0f ',schedule_start_col,schedule_end_col,i,points_num)
          %q(i,:)

        end

    else

        % decide how many points to generate 
        tj=time_schedule(row_tacc,schedule_start_col);
        tk=time_schedule(row_tacc,schedule_end_col);

        t_theta_j=time_schedule(row_time,schedule_start_col);
        t_theta_k=time_schedule(row_time,schedule_end_col);
        
        
        start_time=t_theta_j+0.5*tj;

        
        if schedule_end_col==size(time_schedule,2)
            line_end_time=t_theta_k-tk;
            end_time=t_theta_k;
        else
            line_end_time=t_theta_k-0.5*tk;
            end_time=t_theta_k+0.5*tk;
        end
        
        points_num=(end_time-start_time)/tstep;

        if mod(points_num,1)==0
            points_num=floor(points_num);
        else
            points_num=floor(points_num)+1;
        end
        
        
        q=zeros(points_num,6);
        qd=zeros(points_num,6);
        qdd=zeros(points_num,6);

        q_j=joint_schedule(schedule_start_col,:);

        qd_jk=joint_velocity_schedule(schedule_end_col,:);

        qdd_k = joint_acc_schedule(schedule_end_col,:);
        
   
        t=start_time;
        for i =1:points_num

            
            if i==points_num
                t=end_time;
            else
                t=t + tstep;
            end
            
            
            
            if t<=line_end_time
                q(i,:)=q_j+qd_jk*(t-t_theta_j);
                qd(i,:)=qd_jk;
                qdd(i,:)=0;
                
                % sprintf('From point %.0f to point %.0f  parabolic subpoint %.0f  of %.0f ',schedule_start_col,schedule_end_col,i,points_num)
                 %q(i,:)
                
             

                    
                
            else
                q(i,:)=q_j+qd_jk*(t-t_theta_j)+0.5*qdd_k*(t-t_theta_k+0.5*tk)^2; 
                qd(i,:)=qd_jk+qdd_k*(t-t_theta_k+0.5*tk);
                qdd(i,:)=qdd_k; 
                
                 %sprintf('From point %.0f to point %.0f  linear subpoint %.0f  of %.0f ',schedule_start_col,schedule_end_col,i,points_num)
                 %q(i,:)                %test

                
            end
            

            
            
        end
    end
end



function [time_schedule,joint_velocity_schedule,joint_acc_schedule,joint_schedule] = get_schedule(cartesian_DH,target_velocity,current_joint_variables,tacc)
    %schedule col:time distance tacc

   total_dist=0;
   points_num=size(cartesian_DH,3);
   
   
   row_distance=1;
   row_time=2;
   row_tacc=3;
   time_schedule=zeros(3,points_num);
   
   % add distance
   for i =1:points_num
       if i ==1
            time_schedule(row_distance,i)=0;
       else
           x1=cartesian_DH(1,4,i-1);
           y1=cartesian_DH(2,4,i-1);
           z1=cartesian_DH(3,4,i-1);
           pos1=[x1 y1 z1];
           x2=cartesian_DH(1,4,i);
           y2=cartesian_DH(2,4,i);
           z2=cartesian_DH(3,4,i);
           pos2=[x2 y2 z2];    
           total_dist=total_dist+norm(pos2-pos1);
           time_schedule(row_distance,i)=total_dist;
       end
   end
   

   
   
   
   % calculate total time
   total_time=floor(total_dist/target_velocity);
   

   % add time
   for i =1:points_num
       
       if i==1
           time_schedule(row_time,i)=0;
       elseif i==points_num
           time_schedule(row_time,i)=total_time;
       else
           accu_dist=time_schedule(row_distance,i);
           time_schedule(row_time,i)=floor(accu_dist/total_dist*total_time);
       end
   end

    % add tacc
   for i =1:points_num
       if i==1 || i==points_num
           time_schedule(row_tacc,i) = tacc*0.5;
       else
           time_schedule(row_tacc,i) = tacc;
       end
   end
   
    %joint schedule
    joint_schedule=zeros(size(time_schedule,2),6);
    for i=1:size(time_schedule,2)
        
        [valid,joint_variables]=inverse_kinematics_one(cartesian_DH(:,:,i),current_joint_variables);
        if valid 
            joint_schedule(i,:)=joint_variables;
            current_joint_variables=joint_variables;
        else
            error('fail to get ik %.0f',i)
        end
    end
    
   joint_schedule
   

   

   
   % velocity_schedule
   joint_velocity_schedule=zeros(size(time_schedule,2),6);
   
   for i=2:size(time_schedule,2)
        distance=joint_schedule(i,:)-joint_schedule(i-1,:);
        
        joint_velocity_schedule(i,:)=distance/(time_schedule(row_time,i)-time_schedule(row_time,i-1));
   end
   

   
   % acc schedule
   joint_acc_schedule=zeros(size(time_schedule,2),6);
   
   for i=1:size(time_schedule,2)
       if i==1
           velocity_diff=joint_velocity_schedule(i+1,:)-joint_velocity_schedule(i,:);
           joint_acc_schedule(i,:)=velocity_diff/(0.5*tacc);
       elseif i==size(time_schedule,2)
           velocity_diff=0-joint_velocity_schedule(i,:);
           joint_acc_schedule(i,:)=velocity_diff/(0.5*tacc);
       else
            velocity_diff=joint_velocity_schedule(i+1,:)-joint_velocity_schedule(i,:);
            joint_acc_schedule(i,:)=velocity_diff/tacc;
       end
   end

    % check all schedule have the same dim
    s1=size(time_schedule,2);
    s2=size(joint_velocity_schedule,1);
    s3=size(joint_acc_schedule,1);
    s4=size(joint_schedule,1);
    if s1==points_num && s1==s2 && s2==s3 && s3==s4 
        1;
    else
        error('different size %.0f %.0f %.0f %.0f %.0f',points_num,s1,s2,s3,s4);
    end
    
    
   for i=2:size(time_schedule,2)
        start_acc=0.5*time_schedule(row_tacc,i-1);
        end_acc=0.5*time_schedule(row_tacc,i);
        total_time=time_schedule(row_time,i)-time_schedule(row_time,i-1);
        if (start_acc+end_acc)>total_time
           error('time error between point %.0f and point %.0f',i-1,i) 
        end
    
    end
    

     %joint_schedule

    
end

