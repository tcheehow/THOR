function [theta_con n_con t_check] = controller(x,t,p_input,theta)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global rot_errors a_errors

%%% Adaptive's Errors
P_error = rot_errors(1);
I_error = rot_errors(2);
D_error = rot_errors(3);

%%% Adaptive's Errors
a1_error = a_errors(1);
a2_error = a_errors(2);
a3_error = a_errors(3);

%%% Pilot Input Conversion %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[throttle,roll,pitch,t_check] = time_check(t,p_input);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Updating Errors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

curr_a_error = ((throttle./10)-x(3));
curr_r_error = (65-x(12));

a_errors   = [curr_a_error a_errors(1)+a_errors(2) 0];
rot_errors = [curr_r_error rot_errors(1)+rot_errors(2) 0];

%%% Gains %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

K_1 = 0.06;
K_2 = 0;

K_P = 5;
K_I = 0.03;
K_D = 0;

%%% Flap Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u_v_z = K_1.*a1_error;             % Angle range approx -0.3 to 0.3 at K_1 = 1

u_v_xy = 0;            % Placeholder. Just to keep roll and pitch active.

%%% RPM Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u_n = K_P.*P_error + K_I.*I_error + K_D.*D_error;

if u_n > 330
    u_n = 330;
    debug = 'gains have forced motor RPM to exceed limits, reverting to motor max'
end

%%% Output to Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta_con = u_v_xy + u_v_z;
n_con = u_n;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Diagnostics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

revealer = [t n_con x(12)]