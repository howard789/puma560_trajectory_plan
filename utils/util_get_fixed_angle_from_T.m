function [alpha,beta,gamma] = util_get_fixed_angle_from_T(T)
   if T(1,1)==0 && T(2,1)==0 && T(3,1)<0 
        beta = 90;
        alpha = 0;
        gamma = atan2(T(1,2),T(2,2))*180/pi;
   elseif T(1,1)==0 && T(2,1)==0 && T(3,1)>0 
        beta = -90;
        alpha = 0;
        gamma = -atan2(T(1,2),T(2,2));
    else
        beta=atan2(-T(3,1),sqrt(T(1,1)^2+T(2,1)^2))*180/pi;
        alpha = atan2(T(2,1),T(1,1))*180/pi;
        gamma = atan2(T(3,2),T(3,3))*180/pi;
    end
end