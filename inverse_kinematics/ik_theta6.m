function pack123456 = ik_theta6(pack12345,r11,r21,r31)

    theta1=pack12345(1);
    theta2=pack12345(2);
    theta3=pack12345(3);
    theta4=pack12345(4);
    theta5=pack12345(5);
    
    c1=cos(theta1);
    s1=sin(theta1);
    c23=cos(theta2+theta3);
    c4=cos(theta4);
    s4=sin(theta4);
    s23=sin(theta2+theta3);
    c5=cos(theta5);
    s5=sin(theta5);
    
    s6=-(c1*c23*s4-s1*c4)*r11-(s1*c23*s4)*r21+s23*s4*r31;
    c6=(c1*c23*c4*c5+s1*s4*c5-c1*s23*s5)*r11+(s1*c23*c4*c5-c1*s4*c5-s1*s23*s5)*r21-(s23*c4*c5+c23*s5)*r31;
    
    theta6=atan2(s6,c6);
    
    pack123456=[theta1,theta2,theta3,theta4,theta5,theta6];
end

    
    
    