function [a b] = inflow_func(sigma,Cl_grad,F_p_guess,lambda_c,theta,r)

A = ((sigma.*Cl_grad)./(16.*F_p_guess))-(lambda_c./2);
B = ((sigma.*Cl_grad)./(8.*F_p_guess)).*theta.*r;

C = (A.^2)+B;

if C < 0
%    debug = 'Inflow function imaginary counter triggered'
    k1 = (sigma.*Cl_grad)./(8.*lambda_c.^2);
    k2 = lambda_c-(2.*theta.*r);
    k3 = sqrt((4.*theta.*r).*(theta.*r-lambda_c));
    F_p_guess = k1.*(k2-k3);
    
    a = (lambda_c./2)-((sigma.*Cl_grad)./(16.*F_p_guess));
else
    a = sqrt(C)-A;
end

b = F_p_guess;
        