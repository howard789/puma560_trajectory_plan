function cartesianVector = DH2cartesian( T )
    cartesianVector(1,1)=T(1,4);
    cartesianVector(2,1)=T(2,4);
    cartesianVector(3,1)=T(3,4);
    [alpha,beta,gamma] = util_get_eul_angle_from_T( T );
    cartesianVector(4,1)=alpha;
    cartesianVector(5,1)=beta;
    cartesianVector(6,1)=gamma;
   
end

