function res_T = test_ik_fk(joint_angles)
    addpath(genpath('utils'),genpath('robot'),genpath('inverse_kinematics'),genpath('forward_kinematics'))

    precision=1; %校验的精度 1=0.1 , 2 = 0.01 ,...

    % 测试的时候第五个角度不能是0 否则逆解的时候 theta4 为任意数
    if joint_angles(5)==0
        joint_angles(5)=1;
    end
    
    % 正解笛卡尔坐标位置与姿态
    cartesian_position_DH = forward_kinematics(6,joint_angles)

    
    % 笛卡尔坐标位置与姿态逆解关节角度
    %[joints1,joints2,joints3,joints4,joints5,joints6,joints7,joints8] = inverse_kinematics(cartesian_position_DH);
    [valid,result_list] = inverse_kinematics(cartesian_position_DH);
    
    if valid
        joint_set1=cell2mat(result_list(1)); 
        joint_set2=cell2mat(result_list(2));
        joint_set3=cell2mat(result_list(3));
        joint_set4=cell2mat(result_list(4));
        joint_set5=cell2mat(result_list(5));
        joint_set6=cell2mat(result_list(6));
        joint_set7=cell2mat(result_list(7));
        joint_set8=cell2mat(result_list(8));
        

        % 这8个关节角度的正解应该和刚刚的相同
        compare_same_xyz(forward_kinematics(6,joint_set1),cartesian_position_DH,4,4,'joints1 fk',precision);
        compare_same_xyz(forward_kinematics(6,joint_set2),cartesian_position_DH,4,4,'joints2 fk',precision);
        compare_same_xyz(forward_kinematics(6,joint_set3),cartesian_position_DH,4,4,'joints3 fk',precision);
        compare_same_xyz(forward_kinematics(6,joint_set4),cartesian_position_DH,4,4,'joints4 fk',precision);
        compare_same_xyz(forward_kinematics(6,joint_set5),cartesian_position_DH,4,4,'joints5 fk',precision);
        compare_same_xyz(forward_kinematics(6,joint_set6),cartesian_position_DH,4,4,'joints6 fk',precision);
        compare_same_xyz(forward_kinematics(6,joint_set7),cartesian_position_DH,4,4,'joints7 fk',precision);
        compare_same_xyz(forward_kinematics(6,joint_set8),cartesian_position_DH,4,4,'joints8 fk',precision);
    
        
        % degrees
                
        joint_degree_set1=joint_set1*180/pi
        joint_degree_set2=joint_set2*180/pi
        joint_degree_set3=joint_set3*180/pi
        joint_degree_set4=joint_set4*180/pi
        joint_degree_set5=joint_set5*180/pi
        joint_degree_set6=joint_set6*180/pi
        joint_degree_set7=joint_set7*180/pi
        joint_degree_set8=joint_set8*180/pi      
        
        
        %plot
        
        robot = puma560_robot([0 0 0]);
        subplot(2,4,1);
        title('joint_set1');
        robot.plot(joint_set1);
        
        subplot(2,4,2);
        title('joint_set2');
        robot.plot(joint_set2);
        
        subplot(2,4,3);
        title('joint_set3');
        robot.plot(joint_set3);
        
        subplot(2,4,4);
        title('joint_set4');
        robot.plot(joint_set4);
        
         subplot(2,4,5);
        title('joint_set5');
        robot.plot(joint_set5);
        
        subplot(2,4,6);
        title('joint_set6');
        robot.plot(joint_set6);       
        
         subplot(2,4,7);
        title('joint_set7');
        robot.plot(joint_set7);
        
        subplot(2,4,8);
        title('joint_set7');
        robot.plot(joint_set8);       
    else
       disp('inverse_kinematics invalid ') 
    end
    %current_joints=joint_angles;
    
    %best_joints = find_best_joints_with_min_change(current_joints,result_list)
end


function err = compare_same_xyz(T1,T2,rows,cols,err_msg_prefix,precision)
   err=0;
   for row=1:3

        if T1(row,4) ~= T2(row,4)
            if abs(T1(row,4) - T2(row,4))>0.001;
                err=1;
            end

        end
   end    
   if err==1
       disp(strcat(err_msg_prefix,' err!'))
       disp('T1')
       disp(T1)
       disp('T2')
       disp(T2)
       
   else
       disp(strcat(err_msg_prefix,' ok!'))
   end
   
end




