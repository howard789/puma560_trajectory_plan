function [theta3_1,theta3_2] = ik_theta3(Px,Py,Pz,a2,a3,d3,d4)
    K = (Px^2+Py^2+Pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);
    theta3_1 = atan2(a3,d4) -  atan2(K,  sqrt(d4^2+a3^2-K^2));
    theta3_2 = atan2(a3,d4) -  atan2(K, -sqrt(d4^2+a3^2-K^2));
end