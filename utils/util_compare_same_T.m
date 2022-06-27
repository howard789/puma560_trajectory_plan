function err = util_compare_same_T(T1,T2,rows,cols,err_msg_prefix,precision)
   err=0;
   for i=1:1:rows
        for j=1:1:cols
            if T1(i,j) ~= T2(i,j)
                diff = T1(i,j) - T2(i,j);
                diff = util_close_zero_to_zero(diff,precision);
                if diff ~=0
                    err=1;
                end
                
            end
        end
   end    
   if err==1
       disp(strcat(err_msg_prefix,' err!'))
       disp('T1')
       disp(T1)
       disp('T2')
       disp(T2)
       
   else
       disp(strcat(err_msg_prefix,' ok!'))
   end
   
end

