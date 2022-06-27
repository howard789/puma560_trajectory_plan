function [best_joints,min_change_degs] = find_best_joints_with_min_change(current_joints,results)
    
     min_change_degs=inf;
     best_joints=[];
     for i =1:size(results,2)
         test_joints = cell2mat(results(i));

         [min_change_degs,best_joints] = calculate_distancs_of_joints(i,current_joints,test_joints,min_change_degs,best_joints);
     end
     
     
end


function [min_change_degs,best_joints] = calculate_distancs_of_joints(no,current_joints,test_joints,min_change_degs,best_joints)

    change_degs=0;
    for i=1:3
        change_degs = change_degs+abs(test_joints(i)-current_joints(i));
    end
    
    for i=4:6
        change_degs = change_degs+abs(test_joints(i)-current_joints(i))*0.5;
    end
    
    
    normdis=norm(test_joints-current_joints);
    test_joints
    sprintf('no: %.0f  norm %.2f',no,normdis)
    sprintf('no: %.0f  change_degs %.2f',no,change_degs)

    
    if change_degs<min_change_degs
        best_joints = test_joints;
        min_change_degs=change_degs;
    end
    
end

