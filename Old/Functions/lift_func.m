function a = lift_func(sigma,Cl_grad,theta,r,lambda_out,Resolution)

a = (sigma.*Cl_grad./2).*((theta.*r.*r)-(lambda_out.*r)).*(1./Resolution);

if isnan(a)
    a = 0;          % Not sure if this is acceptable.
end