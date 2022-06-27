function bring_cup_with_matlab_jtraj()
    addpath(genpath('utils'),genpath('robot'),genpath('inverse_kinematics'),genpath('forward_kinematics'),genpath('matlab_trajectory_plan'),genpath('plot'))

    %%将各个点位的杯子target的坐标轴转换末端执行器的坐标轴

    T6_c=[0 0 1 0;0 -1 0 0;1 0 0 -0.05; 0 0 0 1];
    %T0_6_p1 = util_get_end_joint_matric(0,0,35,550,270,195,T6_c);
    %T0_6_p2 = util_get_end_joint_matric(0,0,35,550,270,79.5,T6_c);
    %T0_6_p3 = util_get_end_joint_matric(0,-60,0,330,372,367,T6_c);
    %T0_6_pf = util_get_end_joint_matric(0,-60,0,330,472,367,T6_c);
    
    %alpha,beta,gamma,x,y,z
    T0_6_p1 = util_get_end_joint_matric(0,0,35,     -0.6,-0.4,-0.5,  T6_c);
    T0_6_p2 = util_get_end_joint_matric(0,0,35,     -0.6,-0.4,0,     T6_c);
    T0_6_p3 = util_get_end_joint_matric(0,-60,0,     0.6,-0.4,0,    T6_c);
    T0_6_pf = util_get_end_joint_matric(0,-60,0,     0.6,-0.4,-0.2, T6_c);
    
    [valid,result_list] = inverse_kinematics(T0_6_p1)

    
    %qlim1=[-160, 160]/180*pi;
    %qlim2=[-225, 45]/180*pi;
    %qlim3=[-45, 225]/180*pi;
    %qlim4=[-110, 170]/180*pi;
    %qlim5=[-100, 100]/180*pi;
    %qlim6=[-266, 266]/180*pi;
    
    
    
    %转成cartesianVector,将需求转换成标准的格式,这步之前所有机械臂都是一样的
    %cartesianVector_p1 = get_cartesianVector( T0_6_p1 );
    %cartesianVector_p2 = get_cartesianVector( T0_6_p2 );
    %cartesianVector_p3 = get_cartesianVector( T0_6_p3 );
    %cartesianVector_p4f = get_cartesianVector( T0_6_pf );
    
    % 仿真
    cartesian_points{1}=T0_6_p1;
    cartesian_points{2}=T0_6_p2;
    cartesian_points{3}=T0_6_p3;
    cartesian_points{4}=T0_6_pf;
    
    
   base_xyz=[0,0,0];
   robot=puma560_robot(base_xyz); 
   
   [valid,result_list] = inverse_kinematics(T0_6_p1);
   if valid 
        start_joints=cell2mat(result_list(1));
   else
       disp('point 1 is invalid');
   end

   %generate 5 sub-points for each stage(when moving on the line between two points), the number of total points should
   %be 1(first point)+4(step-1)*3(3 stages between 4 points)=13;
   step=30;
    
  % joints轨迹规划,需要先运行startup_rvc才能调用get_jtraj
   [valid_1,joint_q,joint_qd,joint_qdd] =  get_jtraj(start_joints,cartesian_points,step);
    
   %笛卡尔坐标系规划
   [valid_2,Tc_list] = get_ctraj(cartesian_points,step);
   
   
   Tc=zeros(4,4,size(Tc_list,2));
   for i=1:size(Tc_list,2)
        Tc(:,:,i)=cell2mat(Tc_list(i));
   
   end
   
   
   DH_points=zeros(4,4,size(cartesian_points,2));
   for i =1:size(cartesian_points,2)
        DH_points(:,:,i)=cell2mat(cartesian_points(i));
   
   end
   
   
   if valid_1 && valid_2
       plot_data(robot,joint_q,joint_qd,joint_qdd,Tc,DH_points);
       
   else
      disp('can not plan'); 
   end

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
end



