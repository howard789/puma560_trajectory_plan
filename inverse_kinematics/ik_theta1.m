function [theta1_1,theta1_2] = ik_theta1(Px,Py,d3)
    theta1_1 = atan2(Py,Px) -  atan2(d3,  sqrt(Px^2+Py^2-d3^2));
    theta1_2 = atan2(Py,Px) -  atan2(d3, -sqrt(Px^2+Py^2-d3^2));
end
