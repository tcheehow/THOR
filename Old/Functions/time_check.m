function [a,b,c,d] = time_check(t,p_input)

count = size(p_input,1);

for k = 1 : (count-1)
    if t >= p_input(k,1) && t < p_input(k+1,1)
        a = p_input(k,2);
        b = p_input(k,3);
        c = p_input(k,4);
        d = k;
    end
    if t == p_input(end,1)
        a = p_input(end,2);
        b = p_input(end,3);
        c = p_input(end,4);
        d = k;
    end
end
