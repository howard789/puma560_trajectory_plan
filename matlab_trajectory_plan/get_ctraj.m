function [valid,Tc_list] = get_ctraj(cartesian_points,step)
    
    T0=cell2mat(cartesian_points(1));    
    Tc_list{1,1}=T0;
    for i =2:size(cartesian_points,2)
        T1=cell2mat(cartesian_points(i));
        Tc = ctraj(T0,T1,step);
        for j = 2:step
            Tc_one=Tc(:,:,j);
            Tc_list{1,end+1}=Tc_one;
        end
        T0=T1;
    end
    
    %check all points are reachable
    [alpha_data,a_data,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = puma560_dh();
    d3=d(3);
    d4=d(4);
    a2=a_data(3);
    a3=a_data(4);


   
    for i =1:size(Tc_list,2)
       Tc= Tc_list{1,i};
       valid =  ik_valid(Tc,a2,a3,d3,d4);
       if ~valid
          disp('get_ctraj: can not reach point:');
          i
          break;
       end
    end

end