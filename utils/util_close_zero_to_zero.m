function x=util_close_zero_to_zero(x,precision)
    thre=1;
    for i=1:1:precision
        thre = thre/10;
    end

    if x>-thre && x<thre
        x = 0;
    end
    
end