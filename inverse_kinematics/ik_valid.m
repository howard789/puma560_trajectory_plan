function valid= ik_valid(cartesian_position_DH,a2,a3,d3,d4)

    if size(cartesian_position_DH,1)==4 && size(cartesian_position_DH,2)==4 && size(cartesian_position_DH,3)==1
        1;
    else
        error('ik_valid size err');
    end
    
    Px = cartesian_position_DH(1,4);
    Py = cartesian_position_DH(2,4);
    Pz = cartesian_position_DH(3,4);
    

    
    if (Px^2+Py^2-d3^2)<0
        disp('can not get theta1. point is too closed');
        valid =0;

        
        while 1
            if Px<0
                Px=Px-0.01;
            else
                Px=Px+0.01;
            end
            if Py<0
                Py=Py-0.01;
            else
                Py=Py+0.01;
            end    
            if (Px^2+Py^2-d3^2)>0
                disp('recommend Px:%.2f ,Py:%.2f',Px,Py)
                break;
            end 
        end

    elseif a3^2+d4^2-((Px^2+Py^2+Pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2))^2<0        
        disp('can not get theta3')
        valid =0;
    elseif sqrt(Px^2+Py^2+Pz^2)>(a2+d4)
        % max length should be a little bigger than a2+d4
        disp('out of range');
        valid =0;
    else
        valid =1;
    end
    
    
    
    
    
    
    
end