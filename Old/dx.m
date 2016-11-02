function [dx wing_effects] = dx(t,x,scalars, wing, prop_data, airfoil_data, p_input, x_motor, I)

% Notes:
% #1 Should record the progression of wing forces over time as a matrix
% #2 Need to account for Coning Angle
% #3 In what direction does dL really operate at? In the paper I wrote that
%    since U_T is very high, the difference between dFz and dL would be very
%    small.
% #4 Might want to clarify the differences between theta, theta_con, alpha
%    etc.
% #5 The AoA (limit check) keeps proccing. Need to correct it.
% #6 It seems that once the fs_velocity passing through the prop exceeds a
%    certain number, the Ct drops to 0. The current Ct_finder assumes so.
%    If so, it could mean that after reaching a certain rpm, i might as
%    well turn off the rotor.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% State Space %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x(1)    Inertial Frame Displacement X
% x(2)    Inertial Frame Displacement Y
% x(3)    Inertial Frame Displacement Z
% x(4)    Inertial Frame Angle X
% x(5)    Inertial Frame Angle Y
% x(6)    Inertial Frame Angle Z
% x(7)    Body Frame Linear Velocity X
% x(8)    Body Frame Linear Velocity Y
% x(9)    Body Frame Linear Velocity Z
% x(10)   Body Frame Angular Velocity X
% x(11)   Body Frame Angular Velocity Y
% x(12)   Body Frame Angular Velocity Z

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Unpacking Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Scalars
Resolution   = scalars(1);
m            = scalars(2);
R            = scalars(3);
N_b          = scalars(4);
D            = scalars(5);
rho          = scalars(6);
g            = scalars(7);
Cl_grad      = scalars(8);
Cl_lower     = scalars(9);
Cl_upper     = scalars(10);
Cl_optimal   = scalars(11);

%%% Wing Stuff
r            = wing(:,1)';
sigma        = wing(:,2);
chord        = wing(:,3);
theta        = wing(:,4);

%%% ODE45 Outputs
dx         = zeros(12,1);

dL         = zeros(Resolution,1);
dD         = zeros(Resolution,1);
AoA        = zeros(Resolution,1);
v_i        = zeros(Resolution,1);

% Note: Ct is also output via the ODE but we don't need to declare it
% here.

%%% Other Outputs
lambda     = zeros(Resolution,1);
dC_Tn      = zeros(Resolution,1);
F_check    = zeros(Resolution,2);

global t_check

%%% Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[theta_con n_con t_check] = controller(x,t,p_input,theta);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Forces and Moments %%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta_con_w = theta + theta_con;  % applies flap controller across the wing

% Forces and Moments are takens w.r.t the aircraft frame.

%%% Motor %%%%%
if n_con <= 75
    n_con = 80;  % no data available for spin rates below 75. So I assume arming would cause motor to spin at 75 rev/s.
end

col          = 3.*(round(3.*(n_con-75)./50)+1)-2;
prop_fs_list = prop_data(5:34,col);          
Ct_list      = prop_data(5:34,(col+1));

prop_fs      = x(12).*x_motor(1)+x(8);
Ct           = Ct_finder(prop_fs_list,Ct_list,prop_fs);

fMotor = [0 ; (Ct.*rho.*(n_con.^2).*D.^4) ; 0];
mMotor = [0 ; 0 ; x_motor(1).*fMotor(2)];

%%% Lift %%%%%
if x(12) == 0
    lambda_c = 0;
else
    lambda_c = x(9)./(R.*x(12));
end

for j = 1 : Resolution
    % We assume F = 1 for initial calculation of lambda
    F_check(j,1) = 1;
    F_check(j,2) = 0;
    [lambda_guess F_check(j,1)] = inflow_func(sigma(j),Cl_grad,F_check(j,1),lambda_c,theta_con_w(j),r(j)); 
    
    while abs(F_check(j,1)-F_check(j,2)) > 0.01
        F_check(j,2)  = F_check(j,1);
        F_check(j,1)  = prandtl_func(N_b,r(j),lambda_guess);
        lambda_guess  = inflow_func(sigma(j),Cl_grad,F_check(j,1),lambda_c,theta_con_w(j),r(j));
    end
    
    AoA(j) = theta_con_w(j) - atan(lambda_guess);
    
    if AoA(j)  > Cl_upper || AoA(j) < Cl_lower
% Change below to 1 to activate Cl linear limit checker. I turned it off
% because the results would produce insufficient lift. The checker in
% itself is crude because it acts as if no lift is generated in the
% non-linear regions. This is not true, rather the lift generated is simply
% less.
        wing_wasted(j) = 1;
    else
        wing_wasted(j) = 0;
    end
    
    lambda(j) = lambda_guess;
    v_i(j)    = (lambda(j).*(x(12).*R))-x(9);
end

for k = 1 : Resolution
    if wing_wasted(k) == 0
        dC_Tn(k) = lift_func(sigma(k),Cl_grad,theta_con_w(k),r(k),lambda(k),Resolution);
    else
        %Trying something out here where, if it exceeds the linear region,
        %I degenerate the Cl_grad to a lower number.
        dC_Tn(k) = lift_func(sigma(k),2.6,theta_con_w(k),r(k),lambda(k),Resolution);
    end
end

for k = 1 : Resolution
    dL(k) = dC_Tn(k).*rho.*(pi.*(R.^2)).*((x(12).*R).^2);
end 

fLift = [0 ; 0 ; sum(dL)];
mLift = [0 ; 0 ; 0];

%%% Drag %%%%%

for k = 1 : Resolution
    dD(k) = dL(k).*drag_func(AoA(k),airfoil_data);
end

fDrag = [0 ; -sum(dD) ; 0];
mDrag = [0 ; 0 ; sum(-dD.*r'.*R)];

%fDrag = [0 ; 0 ; 0];
%mDrag = [0 ; 0 ; 0];

%%% Gravity %%%%%
fGrav = inv(Euler2R([x(4) x(5) x(6)]))*[0 ; 0 ; m.*g];
mGrav = [0 ; 0 ; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% State Space %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

b_F = fMotor + fLift + fDrag + fGrav;
b_M = mMotor + mLift + mDrag + mGrav;

i_l_vel = Euler2R([x(4) ; x(5) ; x(6)])*[x(7) ; x(8) ; x(9)]; 
i_a_vel = Euler2R([x(4) ; x(5) ; x(6)])*[x(10) ; x(11) ; x(12)];
b_l_acc = (1./m).*b_F;
b_a_acc = inv(I)*b_M;

dx = vertcat(i_l_vel,i_a_vel,b_l_acc,b_a_acc);

wing_effects = horzcat(dL', dD', AoA', v_i', Ct);

assignin('base','input_cons',[theta_con n_con]);
evalin('base','proc_inputs(t_check,:) = input_cons;');

assignin('base','aero_forces',[fLift' fDrag']);
evalin('base','forces(t_check,:) = aero_forces;');

assignin('base','wing_out',wing_effects);
evalin('base','wing_effects(end+1,:)=wing_out;');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graphing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% For every iteration it is useful to know the 12 state variables,
% individual forces and controller feedback. Some of these should be
% graphed (state variables and controller feedback) and some should be
% visualized (direction of forces).
%
% Other variables to track would be information along the wing... dL, F,
% dD, AoA, Ct.
%
% This also means that the data would need to be stored somewhere as a
% function of time.