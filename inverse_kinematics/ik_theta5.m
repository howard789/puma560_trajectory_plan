function joints12345 = ik_theta5(joints1234,r13,r23,r33)
    theta1=joints1234(1);
    theta2=joints1234(2);
    theta3=joints1234(3);
    theta4=joints1234(4);

    c1=cos(theta1);
    s1=sin(theta1);
    c23=cos(theta2+theta3);
    c4=cos(theta4);
    s4=sin(theta4);
    s23=sin(theta2+theta3);
    
    s5=(-c1*c23*c4-s1*s4)*r13-(s1*c23*c4-c1*c4)*r23+s23*c4*r33;
    c5=-c1*s23*r13-s1*s23*r23-c23*r33;
    
    theta5=atan2(s5,c5);
    
    joints12345=[theta1,theta2,theta3,theta4,theta5];

end