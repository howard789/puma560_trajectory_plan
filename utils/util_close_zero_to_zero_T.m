function T=util_close_zero_to_zero_T(T,rows,cols,precision)

    for i=1:1:rows
        for j=1:1:cols
            T(i,j)=util_close_zero_to_zero(T(i,j),precision);
        end
    end    
end