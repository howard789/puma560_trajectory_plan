function pack123456_1 = ik_double_theta456(pack123456)
    theta1=pack123456(1);
    theta2=pack123456(2);
    theta3=pack123456(3);
    theta4=pack123456(4);
    theta5=pack123456(5);
    theta6=pack123456(6);
    
    
    theta4_2=theta4+pi;
    theta5_2=-theta5;
    theta6_2=theta6+pi;
    
    if theta4_2>=2*pi
        theta4_2=theta4_2-2*pi;
    end
    
    if theta6_2>=2*pi
        theta6_2=theta6_2-2*pi;
    end
    
    pack123456_1=[theta1,theta2,theta3,theta4_2,theta5_2,theta6_2];
    



end