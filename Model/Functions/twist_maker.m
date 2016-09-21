function a = twist_maker(type,r,var)
% Builds the twist of the blade elements across the wing

% Type 0: Linear where var = (start length, gradient)
% Type 1: Reciprocal where var = (reciprocal)
% Type 2: Hybrid where the twist is constant from 0 to 0.3 of the wing and
% from 0.35 to 1 it is reciprocal

if type == 0
    a = var(1) + (var(2) * r);

elseif type == 1
    a = var./r;
    
elseif type == 2
    for k = 1 : length(r)
        if r(k) <= 0.3
            a(k) = var./0.3;
        else
            a(k) = var./r(k);
        end
    end
    
end
        
