function [alpha,beta,gamma] = util_get_eul_angle_from_T( T )
    if T(1,3)==0 && T(2,3)==0 && T(3,3)>0
        beta = 0;
        alpha = 0;
        gamma = atan2(-T(1,2),T(1,1))*180/pi;
    elseif T(1,3)==0 && T(2,3)==0 && T(3,3)<0   
        beta = 180;
        alpha = 0;
        gamma = atan2(T(1,2),-T(1,1))*180/pi;
    else
        beta=atan2(sqrt((T(3,1)^2)+(T(3,2)^2)),T(3,3))*180/pi;
        alpha = atan2(T(2,3),T(1,3))*180/pi;
        gamma = atan2(T(3,2),-T(3,1))*180/pi;
    
    end
    eul = tr2eul(T)*180/pi;

end

