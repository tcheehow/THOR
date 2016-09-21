function a = chord_maker(type,r,var)
% Builds the chord across the wing span

% Type 0: Linear where var = (start length, gradient)

if type == 0
    a = var(1) + (var(2) * r);

elseif type == 1
    a = var./r;    
end
