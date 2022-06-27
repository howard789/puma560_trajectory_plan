function bring_cup_with_my_trajectory()
    addpath(genpath('utils'),genpath('robot'),genpath('inverse_kinematics'),genpath('forward_kinematics'),genpath('my_trajectory_plan'),genpath('plot'))

    addpath(genpath('utils'))
    
    use_cartesian_plan=0;
    
    
    %%将各个点位的杯子target的坐标轴转换末端执行器的坐标轴

    T6_c=[0 0 1 0;0 -1 0 0;1 0 0 -0.05; 0 0 0 1];
    %T6_c=[1 0 0 0;0 1 0 0;0 0 1 0; 0 0 0 1];
    
    %alpha,beta,gamma,x,y,z       cartesian fail and Joints_plan ok
    T0_6_p1 = util_get_end_joint_matric(0,0,35,     -0.6,-0.2,0,     T6_c);
    T0_6_p2 = util_get_end_joint_matric(0,0,35,     -0.6,-0.4,-0.4,   T6_c);
    T0_6_p3 = util_get_end_joint_matric(0,0,35,     -0.6,-0.4,0,      T6_c);
    T0_6_p4 = util_get_end_joint_matric(0,-60,0,     0.5,-0.4,0.4,    T6_c);
    T0_6_p5 = util_get_end_joint_matric(0,-60,0,     0.5,-0.4,0.1,    T6_c);
    
    
    %T0_6_p1 = util_get_end_joint_matric(0,0,0,     -0.4,-0.2,0,     T6_c);
    %T0_6_p2 = util_get_end_joint_matric(0,0,0,     -0.4,-0.3,-0.2,   T6_c);
    %T0_6_p3 = util_get_end_joint_matric(0,0,0,     -0.4,-0.4,0,      T6_c);
    %T0_6_p4 = util_get_end_joint_matric(0,0,0,     0.3,-0.3,0.3,    T6_c);
    %T0_6_p5 = util_get_end_joint_matric(0,0,0,     0.3,-0.3,0.1,    T6_c);
    
    
    
    points_num=5;
    DH_points=zeros(4,4,points_num);
    DH_points(:,:,1)=T0_6_p1;
    DH_points(:,:,2)=T0_6_p2;
    DH_points(:,:,3)=T0_6_p3;
    DH_points(:,:,4)=T0_6_p4;
    DH_points(:,:,5)=T0_6_p5;
    
    target_velocity = 0.05; % 1cm/sec
    tstep = 0.1;
    
    tacc =3;
    
    base_xyz=[0,0,0];
    robot=puma560_robot(base_xyz); 

    
    
    if use_cartesian_plan
        [q,qd,qdd] = get_cartesian_trajectory_plan(DH_points,target_velocity,tstep,tacc);
        
        current_joint_variables=[0 0 0 0 0 0];
        [valid,start_joints]=inverse_kinematics_one(DH_points(:,:,1),current_joint_variables);
        
        [valid,joint_q,joint_qd,joint_qdd] = get_joint_data(start_joints,q,tstep);

        Tc=q;
    else
      

        [valid,joint_q,joint_qd,joint_qdd] = get_joints_trajectory_plan(DH_points,target_velocity,tstep,tacc);
        q=zeros(4,4,size(joint_q,1));
        for i=1:size(joint_q,1)
            %joint_rads=joint_q(i,:)/180*pi;
            T = forward_kinematics(6,joint_q(i,:));
            q(:,:,i)=T;
        end
        
        Tc=q;
    end
    
    
    

    if valid 
        plot_data(robot,joint_q,joint_qd,joint_qdd,Tc,DH_points);
    else 
        disp('err to get joint data');
    end
    
    
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
    
   

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
end



