function pack1234=ik_theta4(pack123,r13,r23,r33)

    %[theta1,theta2,theta3] = pack123;
    
    theta1=pack123(1);
    theta2=pack123(2);
    theta3=pack123(3);
    c23=cos(theta2+theta3);
    s23=sin(theta2+theta3);
    c1=cos(theta1);
    s1=sin(theta1);
    valueS = -s1*r13-c23*r23;
    valueC = - c1*c23*r13 - s1*c23*r23 + s23*r33;
    
    
    if valueS<0.0001 && valueS>-0.0001 
        valueS_close_zero = 1;
    else
        valueS_close_zero = 0;
    end
    
        
   if valueC<0.0001 && valueC>-0.0001 
        valueC_close_zero = 1;
    else
        valueC_close_zero = 0;
   end
   
        
        
        
        
    if valueS_close_zero==1 &&  valueC_close_zero==1
        theta4 = 0; %可以取任意值,一般取当前值
    else
        theta4 = atan2(valueS,valueC);
    end
    
    pack1234=[theta1,theta2,theta3,theta4];
end