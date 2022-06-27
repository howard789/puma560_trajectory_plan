function [flg]=inverse2(x,y,z)
    % 用matlab的函数确定是否可以到达

    
    
    [alpha,a,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = dh_data();
    %% link
    L(1) = Link([joint_rads(1),d(1),a(1),alpha(1)],'modified','qlim',qlim1);
    L(2) = Link([joint_rads(2),d(2),a(2),alpha(2)],'modified','qlim',qlim2);
    L(3) = Link([joint_rads(3),d(3),a(3),alpha(3)],'modified','qlim',qlim3);
    L(4) = Link([joint_rads(4),d(4),a(4),alpha(4)],'modified','qlim',qlim4);
    L(5) = Link([joint_rads(5),d(5),a(5),alpha(5)],'modified','qlim',qlim5);
    L(6) = Link([joint_rads(6),d(6),a(6),alpha(6)],'modified','qlim',qlim6);

    
   

    %% robot
    robot = SerialLink(L,'name','puma560');
    posi=[0 0 0 x;0 0 0 y;0 0 0 z;0 0 0 1];
    the=robot.ikine(posi,'mask',[1 1 1 0 0 0],'ilimit',20,'tol',0.01,'slimit',5);
    if isempty(the)
        flg=0;
    else
        flg=1;
    end
end