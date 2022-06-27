function T= util_get_target_matric(alpha,beta,gamma,x,y,z)
    %eur2r_zyz

    alpha_rad=alpha/180*pi;
    beta_rad=beta/180*pi;
    gamma_rad=gamma/180*pi;

    r11=cos(alpha_rad)*cos(beta_rad)*cos(gamma_rad)-sin(alpha_rad)*sin(gamma_rad);
    r12=-cos(alpha_rad)*cos(beta_rad)*sin(gamma_rad)-sin(alpha_rad)*cos(gamma_rad);
    
    r13=cos(alpha_rad)*sin(beta_rad);
    r21=sin(alpha_rad)*cos(beta_rad)*cos(gamma_rad)+cos(alpha_rad)*sin(gamma_rad);
    r22=-sin(alpha_rad)*cos(beta_rad)*sin(gamma_rad)+cos(alpha_rad)*cos(gamma_rad);
    r23=sin(alpha_rad)*sin(beta_rad);
    
    r31=-sin(beta_rad)*cos(gamma_rad);
    r32=sin(beta_rad)*sin(gamma_rad);
    r33=cos(beta_rad);
    
    
    T=[r11 r12 r13 x;r21 r22 r23 y;r31 r32 r33 z; 0 0 0 1;];
    
   % test=eul2r(alpha,beta,gamma);

    
end