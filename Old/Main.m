clear; close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Description %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% So this is the first full rewrite of the monocopter simulation code. The
% previous versions are just too messy for me to be able to effectively
% debug and clean up... There are unused functions, inconsistent notation
% and equations which I'm not sure if they have been corrected etc.

% The purpose of this script is to package all the necessary inputs and to
% run it through the ODE45. So let's get started!!!

% Notes
% #1 Account for the non-linear region of Cl/Alpha (rewrite the equation to
% follow the graph and not the gradient).
% #2 Find a way to add two motors
% #3 Blade Independent Control. This would mean if a blade element fails or
% goes imaginary, we account for it without stopping the entire
% calculation.
% #4 Why the hell does my ideal hover twist start at an absurdly low angle?
% It seems like the range can go much higher and operate more efficiently.
% WHY??!!!!?!!
% #5 Drag uses temporary solution for high AoAs. This is because near root
% we have those ridiculous angles. I think this isn't such a big problem
% because their drag/lift contribution should be minimal. Possible solution
% would be to use a more sensible twist near root.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Craft Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% General %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Craft_Name   = 'Flat_Plate_Dual_250g'; % Craft Name
Resolution   = 30;                               % Number of blade elements
m            = 0.02;                            % Mass of Craft (kg)
R            = 0.24;                             % Wing Radius (m)
N_b          = 1;                                % Number of wings
% I            = [0.00021, -0.00024,  -0.00003;    %
%                 0.00024,  0.00283,  -0.00001;    % Inertia Tensor (kg m^2)
%                 0.00003, -0.00001,   0.00303];
            
I            = [8341.25E-9, 6209.87E-9,  3198.73E-9;    %
                6208.87E-9,  36619.59E-9,  558.27E-9;    % Inertia Tensor (kg m^2)
                3198.73E-9, 558.27E-9,   43807.59E-9];            
            
D            = 0.12065;                          % Propeller Diameter (m)     
x_motor      = [0.16 ; 0 ; 0];                   % Distance Between Motor and Centre of Mass (m)        
prop_file    = 'Database/475x475E';                       % Database file to pull prop data from
airfoil_file = 'Database/NACA0012_NCrit5_Re_400000';      % Database file to pull airfoil data from
ochord_file  = 'Database/chord_optimised1';
Cl_grad      = 6.2849;                           % We assume wing operates along linear Cl/Alpha region
Cl_upper     =  0.3490;                         % Upper angle limit of Cl_grad assumption
Cl_lower     = -0.3490;                         % Lower angle limit of Cl_grad assumption
Cl_optimal   =  0;                         % Optimal point of Cl_grad assumption

r              = linspace((1./Resolution),1,Resolution);
prop_data      = xlsread(prop_file,1);
airfoil_data   = xlsread(airfoil_file,'Main','A3:D335');
chord_data     = xlsread(ochord_file,'A3:A241');
chord_data_tmp = ones(1, Resolution);
chord_ratio    = round(length(chord_data)/ Resolution);
chord_scaler   = R;

for i = 1:length(chord_data_tmp)
    tmp = (i-1)*chord_ratio;
    
    if (tmp > length(chord_data))
        tmp = length(chord_data);
    elseif (tmp <= 0)
        tmp = 1;
    end
    
    chord_data_tmp(i) = chord_data(tmp);
end
chord_data_tmp = chord_data_tmp .* chord_scaler;

folder         = 'output/';
%folder         = '/Users/Administrator/Desktop/Base VIII/Output/';
reference      = strcat(folder,'Reference_Signals.csv');
output_array   = strcat(folder,Craft_Name,'State_Space_Array.csv');
output_graph_1 = strcat(folder,Craft_Name,'State_Space_Graphs.fig');
output_graph_2 = strcat(folder,Craft_Name,'Wing_Effects.fig');
output_wing    = strcat(folder,Craft_Name,'Wing_Effects.csv');

%%% Chord Taper %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
type         = 2;                               % 0 = Linear
param        = [chord_data_tmp];                     % Linear = (start length, gradient)

chord        = chord_maker(type,r,param);       % Builder

%%% Wing Twist %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
type         = 0;                              % 0 = Linear, 1 = Reciprocal, 2 = Hybrid
param        = [-0.1745, 0];                      % Linear = (start angle, gradient), Reciprocal = (tip angle)

theta        = twist_maker(type,r,param);      % Builder
%theta = theta+0.01                             % Added 0.1 to give more downward angle options
%%% Solidity %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sigma        = blades_maker(R,chord,N_b);      % Builder

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% Environmental Constants %%%%%%%%%%%%%%%%%%%%%%%%%

rho          = 1.225;                          % Air Density (kg/m3)
g            = -9.81;                          % Acceleration due to Gravity (m/s^2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% ODEsolver Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%

t_step       = 0.01;                                 % time steps
t_total      = 1.2;                                   % total ODE45 time analyzed
options      = odeset('RelTol',1e-3,'AbsTol',1e-5);  % tolerance options
x0           = [0;0;0;0;0;0;0;0;0;0;0;0];            % initial conditions

tspan        = t_step:t_step:t_total;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Pilot Input Over Time %%%%%%%%%%%%%%%%%%%%%%%%%%

freq         = 100;                               % number of inputs per second
p_time       = linspace(0,t_total,t_total.*freq)';% time setup w.r.t pilot

%%%
dimension = length(p_time);
p_throttle   = 0.*ones(t_total.*100,1);  % throttle input (  0%  <->  100% )                                         
p_throttle(dimension/2:dimension) = 50;

p_roll       = zeros(t_total.*100,1);           % pitch input    ( -50% <->  50%  )
p_pitch      = zeros(t_total.*100,1);           % pitch input    ( -50% <->  50%  )
%%%

p_input      = [p_time p_throttle p_roll p_pitch];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Packing Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scalars      = [Resolution m R N_b D rho g Cl_grad Cl_lower Cl_upper Cl_optimal];
wing         = [r' sigma' chord' theta'];
prop_data    = [prop_data];
airfoil_data = [airfoil_data];
p_input      = [p_input];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ODE Solver %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global a_errors rot_errors t_check

wing_effects = zeros(1,Resolution.*4+1);
proc_inputs  = zeros(size(p_time,1),2);
forces       = zeros(size(p_time,1),6);

init_a_error = ((3.*p_throttle(1)./25)-6)-x0(9); 

t_check      = 1;   
a_errors     = [init_a_error 0 0];
rot_errors   = [x0(3)  0  0];

[t,x] = ode45(@(t,x) dx(t,x,scalars, wing, prop_data, airfoil_data, p_input, x_motor, I),tspan,x0,options);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ref_inputs     = [p_input proc_inputs zeros(size(p_input,1),1)];

h_1 = main_presenter(t,x);

h_2 = wing_presenter(Resolution,r,wing_effects);
savefig(h_1,output_graph_1);

dlmwrite(reference,ref_inputs)
dlmwrite(output_array,[t x forces]);
dlmwrite(output_wing,wing_effects);

