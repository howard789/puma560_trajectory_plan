function joints123 = ik_theta2(theta1,theta3,Px,Py,Pz,a2,a3,d4)
    s23 = ((-a3-a2*cos(theta3))*Pz+(cos(theta1)*Px+sin(theta1)*Py)*(a2*sin(theta3)-d4));
    c23 = ((a3+a2*cos(theta3))*(cos(theta1)*Px+sin(theta1)*Py)+(a2*sin(theta3)-d4)*Pz);
    theta23 = atan2(s23,c23);
    theta2 = theta23-theta3;
    joints123=[theta1,theta2,theta3];
    
end