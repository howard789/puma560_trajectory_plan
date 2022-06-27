function [valid,result_list] = inverse_kinematics(cartesian_position_DH)
    %return 8 sets of rad 
    [alpha_data,a_data,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = puma560_dh();
    d3=d(3);
    d4=d(4);
    a2=a_data(3);
    a3=a_data(4);
    qlim_list={qlim1,qlim2,qlim3,qlim4,qlim5,qlim6};

    valid =  ik_valid(cartesian_position_DH,a2,a3,d3,d4);
    if valid
        [joints1,joints2,joints3,joints4,joints5,joints6,joints7,joints8] = inverse_kinematics_8_result(cartesian_position_DH);
        result_list={joints1,joints2,joints3,joints4,joints5,joints6,joints7,joints8};
        
        
        %result_list= in_robot_joint_limit(qlim_list,joints_list);
        %if size(result_list,1)==0
        %    valid=0;
        %end
        
    else
        result_list={};
    end
end

function results = in_robot_joint_limit(qlim_list,joints_list)
    results=[];
    for i =1:8
        joints = cell2mat(joints_list(i));
        valid=1;
        %4-6 matlab ø¥≤ª¡À
        for j=1:3
            joint = joints(j);
            qlim = cell2mat(qlim_list(j));
            if joint < qlim(1) || joint > qlim(2)
                valid=0;
            end
        end
        
        if valid
            results=[results;joints];
        end
        
        
        
    end
    
    

end

function [joints1,joints2,joints3,joints4,joints5,joints6,joints7,joints8] = inverse_kinematics_8_result(cartesian_position_DH)

    [alpha_data,a_data,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = puma560_dh();
    d3=d(3);
    d4=d(4);
    a2=a_data(3);
    a3=a_data(4);
    
    Px = cartesian_position_DH(1,4);
    Py = cartesian_position_DH(2,4);
    Pz = cartesian_position_DH(3,4);
    
    r11=cartesian_position_DH(1,1);
    %r12=cartesian_position_DH(1,2);
    r13=cartesian_position_DH(1,3);
    
    r21=cartesian_position_DH(2,1);
    %r22=cartesian_position_DH(2,2);
    r23=cartesian_position_DH(2,3);
    
    r31=cartesian_position_DH(3,1);
    %r32=cartesian_position_DH(3,2);
    r33=cartesian_position_DH(3,3);

    
    [theta1_1,theta1_2] = ik_theta1(Px,Py,d3);
    
    [theta3_1,theta3_2] = ik_theta3(Px,Py,Pz,a2,a3,d3,d4);
    
     joints123_1 = ik_theta2(theta1_1,theta3_1,Px,Py,Pz,a2,a3,d4);
     joints123_2 = ik_theta2(theta1_1,theta3_2,Px,Py,Pz,a2,a3,d4);
     joints123_3 = ik_theta2(theta1_2,theta3_1,Px,Py,Pz,a2,a3,d4);
     joints123_4 = ik_theta2(theta1_2,theta3_2,Px,Py,Pz,a2,a3,d4);
     
     joints1234_1=ik_theta4(joints123_1,r13,r23,r33);
     joints1234_2=ik_theta4(joints123_2,r13,r23,r33);
     joints1234_3=ik_theta4(joints123_3,r13,r23,r33);
     joints1234_4=ik_theta4(joints123_4,r13,r23,r33);

     joints12345_1 = ik_theta5(joints1234_1,r13,r23,r33);
     joints12345_2 = ik_theta5(joints1234_2,r13,r23,r33);
     joints12345_3 = ik_theta5(joints1234_3,r13,r23,r33);
     joints12345_4 = ik_theta5(joints1234_4,r13,r23,r33);
     
     joints1 = ik_theta6(joints12345_1,r11,r21,r31);
     joints2 = ik_theta6(joints12345_2,r11,r21,r31);
     joints3 = ik_theta6(joints12345_3,r11,r21,r31);
     joints4 = ik_theta6(joints12345_4,r11,r21,r31);
     
     joints5 = ik_double_theta456(joints1);
     joints6 = ik_double_theta456(joints2);
     joints7 = ik_double_theta456(joints3);
     joints8 = ik_double_theta456(joints4);
     

     
     

end