function T= util_fixed_angle2r(alpha,beta,gamma)
    alpha=alpha/180*pi;
    beta=beta/180*pi;
    gamma=gamma/180*pi;


    r11=cos(alpha)*cos(beta);
    r12=cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);
    
    r13=cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
    
    
    r21=sin(alpha)*cos(beta);
    
    r22=sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
    
    r23=sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
    
    r31=-sin(beta);
    r32=cos(beta)*sin(gamma);
    r33=cos(beta)*cos(gamma);
    
    
    T=[r11 r12 r13;r21 r22 r23;r31 r32 r33;];
    
    test=eul2r(alpha,beta,gamma)
    
end