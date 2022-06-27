function [q,qd,qdd] = get_cartesian_trajectory_plan(DH_points,target_velocity,tstep,tacc)

    if size(DH_points,3)<3
        error('There should be at least 3 points')
    end

    % prepare schedule
    [time_schedule,velocity_schedule,acc_schedule] = get_schedule(DH_points,target_velocity,tacc);

    
    q=DH_points(:,:,1);
    qd=zeros(4,4,1);
    qdd=zeros(4,4,1);
    
    points_num_record=[1];
    for i = 0:size(time_schedule,2)-1

        
       % 求出一个片段的q qd qdd
        schedule_start_col=i;
        schedule_end_col=i+1;
        [q1,qd1,qdd1]=get_trajectory(schedule_start_col,schedule_end_col,time_schedule,velocity_schedule,acc_schedule,DH_points,tstep);
        
        
        sprintf('schedule_start_col %.0f first:',schedule_start_col)
        q1(:,:,1)
        q1(:,:,end)
        
        
        
        points_num_record=[points_num_record;points_num_record(end,1)+size(q1,3)];
        
        
        q=cat(3,q,q1);
        qd=cat(3,qd,qd1);
        qdd=cat(3,qdd,qdd1);
    end
    sprintf('points_num_record:')
    points_num_record
end




function [q,qd,qdd]=get_trajectory(schedule_start_col,schedule_end_col,time_schedule,velocity_schedule,acc_schedule,DH_points,tstep)

   row_distance=1;
   row_time=2;
   row_tacc=3;



    if schedule_start_col==0
        %从第一个点到第1和2的直线段,当schedule_start_col=1时是从1和2的直线段到第2个点曲线段结束
         tk=time_schedule(row_tacc,1);
         points_num=(0.5*tk)/tstep;
         if mod(points_num,1)==0
            points_num=floor(points_num);
        else
            points_num=floor(points_num)+1;
        end
        q=zeros(4,4,points_num);
        qd=zeros(4,4,points_num);
        qdd=zeros(4,4,points_num);
        
        qdd_k = acc_schedule(:,:,1);

        end_time=0.5*tk;
        sprintf('schedule_start_col: %.0f ,start_time: %.2f ,end_time: %.2f',schedule_start_col,0,end_time)
        
        t=0;
        for i =1:points_num
        
          
            if i==points_num
                t=end_time;
            else
                t=t + tstep;
            end
          
          
          q(:,:,i)=DH_points(:,:,1)+0.5*qdd_k*(t+0.5*tk);
          qd(:,:,i)=qdd_k*(t+0.5*tk);
          qdd(:,:,i)=qdd_k; 
          

          
          
          %test
          DH_p=DH_points(:,:,1)+0.5*qdd_k*(t+0.5*tk);
          [valid,result_list] = inverse_kinematics(DH_p);
          if ~valid
            sprintf('From point %.0f to point %.0f  parabolic subpoint %.0f  of %.0f ',schedule_start_col,schedule_end_col,i,points_num)
            q(:,:,i)
            error('err')
          else
              1;
          end
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
        
        %print start_time
        points_num=points_num+1;
        
        
        
        q=zeros(4,4,points_num);
        qd=zeros(4,4,points_num);
        qdd=zeros(4,4,points_num);

        q_j=DH_points(:,:,schedule_start_col);

        qd_jk=velocity_schedule(:,:,schedule_end_col);

        qdd_k = acc_schedule(:,:,schedule_end_col);

        t=start_time-tstep;
        
        
       sprintf('schedule_start_col: %.0f ,start_time: %.2f ,end_time: %.2f',schedule_start_col,start_time,end_time)

        
        for i =1:points_num
            if i==1 || i==points_num
                1;
            end
            
            

            if i==points_num
                t=end_time;
            else
                t=t + tstep;
            end
            
            
            if t<=line_end_time
                q(:,:,i)=q_j+qd_jk*(t-t_theta_j);
                qd(:,:,i)=qd_jk;
                qdd(:,:,i)=0;
                

                
                %test
                DH_p=q_j+qd_jk*(t-t_theta_j);
                [valid,result_list] = inverse_kinematics(DH_p);
              if ~valid
                   sprintf('From point %.0f to point %.0f  parabolic subpoint %.0f  of %.0f ',schedule_start_col,schedule_end_col,i,points_num)
                q(:,:,i)
                   error('err')
              else
                  1;
              end
                

                    
                
            else
                q(:,:,i)=q_j+qd_jk*(t-t_theta_j)+0.5*qdd_k*(t-t_theta_k+0.5*tk)^2; 
                qd(:,:,i)=qd_jk+qdd_k*(t-t_theta_k+0.5*tk);
                qdd(:,:,i)=qdd_k; 
                
           %test
                DH_p=q_j+qd_jk*(t-t_theta_j)+0.5*qdd_k*(t-t_theta_k+0.5*tk);
                [valid,result_list] = inverse_kinematics(DH_p);
                  if ~valid
                    sprintf('From point %.0f to point %.0f  linear subpoint %.0f  of %.0f ',schedule_start_col,schedule_end_col,i,points_num)
                    q(:,:,i)     
                       error('err')
                  else
                      1;
                  end
            end
   
        end
    end
end



function [time_schedule,velocity_schedule,acc_schedule] = get_schedule(DH_points,target_velocity,tacc)
    %schedule col:time distance tacc

   total_dist=0;
   points_num=size(DH_points,3);
   
   
   row_distance=1;
   row_time=2;
   row_tacc=3;
   time_schedule=zeros(3,points_num);
   
   % add distance
   for i =1:points_num
       if i ==1
            time_schedule(row_distance,i)=0;
       else
           x1=DH_points(1,4,i-1);
           y1=DH_points(2,4,i-1);
           z1=DH_points(3,4,i-1);
           pos1=[x1 y1 z1];
           x2=DH_points(1,4,i);
           y2=DH_points(2,4,i);
           z2=DH_points(3,4,i);
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
   
   
   velocity_schedule=zeros(4,4,size(time_schedule,2));
   
   for i=2:size(time_schedule,2)
        distance=DH_points(:,:,i)-DH_points(:,:,i-1);
        
        velocity_schedule(:,:,i)=distance/(time_schedule(row_time,i)-time_schedule(row_time,i-1));
   end
   
   acc_schedule=zeros(4,4,size(time_schedule,2));
   
   for i=1:size(time_schedule,2)
       if i==1
           velocity_diff=velocity_schedule(:,:,i+1)-velocity_schedule(:,:,i);
           acc_schedule(:,:,i)=velocity_diff/(0.5*tacc);
       elseif i==size(time_schedule,2)
           velocity_diff=0-velocity_schedule(:,:,i);
           acc_schedule(:,:,i)=velocity_diff/(0.5*tacc);
       else
            velocity_diff=velocity_schedule(:,:,i+1)-velocity_schedule(:,:,i);
            acc_schedule(:,:,i)=velocity_diff/tacc;
       end
   end

    % check all schedule have the same dim
    s1=size(time_schedule,2);
    s2=size(velocity_schedule,3);
    s3=size(acc_schedule,3);
    
    if s1==points_num && s1==s2 && s2==s3 
        disp('size ok');
    else
        error('different size %.0f %.0f %.0f %.0f',points_num,s1,s2,s3);
    end
    
    for i=2:size(time_schedule,2)
        start_acc=0.5*time_schedule(row_tacc,i-1);
        end_acc=0.5*time_schedule(row_tacc,i);
        total_time=time_schedule(row_time,i)-time_schedule(row_time,i-1);
        if (start_acc+end_acc)>total_time
           error('time error between point %.0f and point %.0f',i-1,i) 
        end
    
    end
    
    
    
    
    
    if 1
        for i = 1:size(velocity_schedule,3)
            i
            vel=velocity_schedule(:,:,i)
            acc=acc_schedule(:,:,i)
        end
    end
    
    time_schedule

end




