%startup_rvc
clear;clc;close all

addpath(genpath('demo'),genpath('my_trajectory_plan'))


%%
% ���⹫ʽ�Ƶ� https://blog.csdn.net/howard789/article/details/123142492
% ��⹫ʽ�Ƶ� https://blog.csdn.net/howard789/article/details/123150942
% �켣�滮�Ĺ�ʽ ̨�������ѧ֮�˶�ѧ��������Ⱥ 
%               https://blog.csdn.net/howard789/article/details/123169040
%               https://blog.csdn.net/howard789/article/details/123170538
% �켣�滮�Ĳο����� https://github.com/cheng0560011/Trajectory-Planning-for-PUMA560


%%
demo_ik_fk=0; %��������
demo_show_forward=0; %������ʾ
demo_show_inverse=0; %λ�������ʾ
demo_bring_cup_with_matlab_jtraj=0;  %����matlab�ķ������й켣�滮
demo_my_trajectory_plan=1;   %�Լ�д�Ĺ켣�滮����

if demo_ik_fk+demo_show_forward+demo_show_inverse+demo_bring_cup_with_matlab_jtraj+demo_my_trajectory_plan>1
    error('run one function one time һ������һ��ѡ��')
end


demo_csdn=0;
%joint_angles=[0 90 90 90 0 0]; err
%robot = puma560_dh(joint_angles,1);
%disp('fowrard_kinematics:')


%% ����inverse kinematics �� forward kinematics
if demo_ik_fk
    %���������Ĵ����Ƿ���ȷ
    %����һ��Ŀ��Ƕ����������ҵ�ĩ��ִ�����ľ����,����8��⣬�ڽ���������⿴�Ƿ���ͬ��
    %���õķ���λ��demo�ļ���
    joint_rads=[50 70 90 20 32 50]/180*pi;
    test_ik_fk(joint_rads);
end

if demo_show_forward
    %ֱ�۵Ĳ鿴�������Ѷ
    %����ؽ���ʾ����ľ�����ʾrobot��״̬
    %���õķ���λ��demo�ļ���
    joints = [30 25 -90 50 70 -80];
    show_forward(joints);
end

if demo_show_inverse
    %ֱ�۵Ĳ鿴������Ѷ
    %����ĩ��ִ�����ľ�����ʾrobot��״̬
    %���õķ���λ��demo�ļ���
    T=[ 0    0.5736    0.8192   -0.3590; ...
         0   -0.8192    0.5736   -0.1784; ...
    1.0000         0         0   -0.0142; ...
         0         0         0    1.0000];
    show_inverse(T);
    
end

if demo_bring_cup_with_matlab_jtraj
    %ʹ��matlab�Ĺ켣�滮
    bring_cup_with_matlab_jtraj();
end

if demo_my_trajectory_plan
    bring_cup_with_my_trajectory();
end


%%