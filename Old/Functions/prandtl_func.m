function a = prandtl_func(N_b,r,lambda)

f = (N_b./2).*(1-r)./(abs(lambda));

a = (2./pi).*(acos(exp(-f)));

if ~isreal(a)
    debug = 'Prandtl factor no longer real'
    keyboard
end
