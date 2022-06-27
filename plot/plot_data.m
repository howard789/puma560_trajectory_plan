function plot_data(robot,joint_q,joint_qd,joint_qdd,Tc,DH_points)
    % Tc 是笛卡尔坐标系的轨迹规划, 一个装满SE3的List,假设有n个点,size=(1xn)
    % robot_q是在笛卡尔坐标系的规划下,用逆运动学求解的角度, joint_q是用角度做的轨迹规划
    s1=size(joint_q,1);
    s2=size(joint_qd,1);
    s3=size(joint_qdd,1);

    s4=size(Tc,3);

    if s1 ~= s2 || s2 ~= s3  || s3 ~= s4 || s1==0
        disp('size err');
    end
    

    %平面中一共分成2*4=8个子画图区间，一共两行，每行四个
    %在第一行第1个子图画位置信息。
        subplot(2,4,1);
        i = 1:6;
        plot(joint_q(:,i)); 
        
        

        
        legend('j1','j2','j3','j4','j5','j6');
        grid on;
        title('位置');
        

    %在第一行第2个子图画速度信息。
        subplot(2,4,2);
        i = 1:6;
        plot(joint_qd(:,i));grid on;
        title('速度');
        

    %在第二行第1个子图画加速度信息。
        subplot(2,4,5);
        i = 1:6;
        plot(joint_qdd(:,i));grid on;
        title('加速度');
        
            
    %在齐次旋转矩阵中提取移动变量，相当于笛卡尔坐标系的点的位置。
    Tjtraj=zeros(size(Tc,3),3);
    for i =1:size(Tc,3)
        Tc_one=Tc(:,:,i);
        Tjtraj(i,1)=Tc_one(1,4);
        Tjtraj(i,2)=Tc_one(2,4);
        Tjtraj(i,3)=Tc_one(3,4);
    end
    
    %Tjtraj是一个n行3列(x y z)的矩阵
    
    
    %在第二行第2个子图画p1到p2直线轨迹。
    subplot(2,4,6);
    plot2(Tjtraj,'r');grid on;
    title('T0到Tf直线轨迹');
    

    
    %在第一行三四子图和第二行三四子图，就相当于整个的右半部分画图
     subplot(2,4,[3,4,7,8]);
     
     %画出轨迹线
     plot2(Tjtraj,'b');
     hold on;
     
     
     %画出必须经过的点
     via_points=zeros(size(DH_points,3),3);
     
     for i = 1:size(DH_points,3)
        via_points(i,1)=DH_points(1,4,i);
        via_points(i,2)=DH_points(2,4,i);
        via_points(i,3)=DH_points(3,4,i);
        
     end
     plot2(via_points,'r>');
     hold on;
     %画出轨迹线
     plot2(Tjtraj,'b');
     hold on;
     
     
     
     robot.plot(joint_q);
     
end
