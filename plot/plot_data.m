function plot_data(robot,joint_q,joint_qd,joint_qdd,Tc,DH_points)
    % Tc �ǵѿ�������ϵ�Ĺ켣�滮, һ��װ��SE3��List,������n����,size=(1xn)
    % robot_q���ڵѿ�������ϵ�Ĺ滮��,�����˶�ѧ���ĽǶ�, joint_q���ýǶ����Ĺ켣�滮
    s1=size(joint_q,1);
    s2=size(joint_qd,1);
    s3=size(joint_qdd,1);

    s4=size(Tc,3);

    if s1 ~= s2 || s2 ~= s3  || s3 ~= s4 || s1==0
        disp('size err');
    end
    

    %ƽ����һ���ֳ�2*4=8���ӻ�ͼ���䣬һ�����У�ÿ���ĸ�
    %�ڵ�һ�е�1����ͼ��λ����Ϣ��
        subplot(2,4,1);
        i = 1:6;
        plot(joint_q(:,i)); 
        
        

        
        legend('j1','j2','j3','j4','j5','j6');
        grid on;
        title('λ��');
        

    %�ڵ�һ�е�2����ͼ���ٶ���Ϣ��
        subplot(2,4,2);
        i = 1:6;
        plot(joint_qd(:,i));grid on;
        title('�ٶ�');
        

    %�ڵڶ��е�1����ͼ�����ٶ���Ϣ��
        subplot(2,4,5);
        i = 1:6;
        plot(joint_qdd(:,i));grid on;
        title('���ٶ�');
        
            
    %�������ת��������ȡ�ƶ��������൱�ڵѿ�������ϵ�ĵ��λ�á�
    Tjtraj=zeros(size(Tc,3),3);
    for i =1:size(Tc,3)
        Tc_one=Tc(:,:,i);
        Tjtraj(i,1)=Tc_one(1,4);
        Tjtraj(i,2)=Tc_one(2,4);
        Tjtraj(i,3)=Tc_one(3,4);
    end
    
    %Tjtraj��һ��n��3��(x y z)�ľ���
    
    
    %�ڵڶ��е�2����ͼ��p1��p2ֱ�߹켣��
    subplot(2,4,6);
    plot2(Tjtraj,'r');grid on;
    title('T0��Tfֱ�߹켣');
    

    
    %�ڵ�һ��������ͼ�͵ڶ���������ͼ�����൱���������Ұ벿�ֻ�ͼ
     subplot(2,4,[3,4,7,8]);
     
     %�����켣��
     plot2(Tjtraj,'b');
     hold on;
     
     
     %�������뾭���ĵ�
     via_points=zeros(size(DH_points,3),3);
     
     for i = 1:size(DH_points,3)
        via_points(i,1)=DH_points(1,4,i);
        via_points(i,2)=DH_points(2,4,i);
        via_points(i,3)=DH_points(3,4,i);
        
     end
     plot2(via_points,'r>');
     hold on;
     %�����켣��
     plot2(Tjtraj,'b');
     hold on;
     
     
     
     robot.plot(joint_q);
     
end
