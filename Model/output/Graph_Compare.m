clear; close all

%%% Graph Comparing Script %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Legend %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Inputs %%%
throttle  = {'alt input ()' 2 [-30 30]};
roll      = {'roll (%)' 3 [-50 50]};
pitch     = {'pitch (%)' 4 [-50 50]};
theta_con = {'flap control (theta)' 5 [-1 1]};
n_con     = {'motor spin (rad/s)' 6 [80 330]};
blank     = {'' 7 [0 100]}; 

%%% Outputs %%%
t       = 1;
P_x     = {'P_x' 'm' 2};
P_y     = {'P_y' 'm' 3};
P_z     = {'P_z' 'm' 4};
theta_x = {'theta_x' 'rad' 5};
theta_y = {'theta_y' 'rad' 6};
theta_z = {'theta_z' 'rad' 7};
v_x     = {'v_x' 'm/s' 8};
v_y     = {'v_y' 'm/s' 9};
v_z     = {'v_z' 'm/s' 10};
omega_x = {'omega_x' 'rad/s' 11};
omega_y = {'omega_y' 'rad/s' 12};
omega_z = {'omega_z' 'rad/s' 13};
F_x    = {'F_x' 'N' 14};
F_y    = {'F_y' 'N' 15};
F_z    = {'F_z' 'N' 16};
D_x    = {'D_x' 'N' 17};
D_y    = {'D_y' 'N' 18};
D_z    = {'D_z' 'N' 19};

%%% Select data to compare %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

target_A1      = P_z;
target_A2      = theta_con;
target_B1      = omega_z;
target_B2      = n_con;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Outputs A %%%
target_A1_title  = target_A1{1,1};
target_A1_units  = target_A1{1,2};
target_A1_column = target_A1{1,3};

%%% Inputs A %%%
target_A2_units  = target_A2{1,1};
target_A2_column = target_A2{1,2};
target_A2_limits = target_A2{1,3};

%%% Output B %%%
target_B1_title  = target_B1{1,1};
target_B1_units  = target_B1{1,2};
target_B1_column = target_B1{1,3};

%%% Inputs B %%%
target_B2_units  = target_B2{1,1};
target_B2_column = target_B2{1,2};
target_B2_limits = target_B2{1,3};
%%%

reference = csvread('Reference_Signals.csv');
time_A2   = reference(:,1);
time_B2   = reference(:,1);

output_A2 = reference(:,target_A2_column);
output_B2 = reference(:,target_B2_column);

flst=dir('*.csv');
flst={flst.name};

ix=regexp(flst,'State_Space_Array');
ix=~cellfun('isempty',ix);

flst=flst(ix);

for k = 1 : size(flst,2) 
    if k == 1
        holder = char(flst(k));
        data = csvread(holder);
        
        time_A1   = data(:,t);
        output_A1 = [data(:,target_A1_column)];
        text   = holder(1:end-22);
        name_A1{1,1} = strrep(text,'_',' ');
    else
        holder = char(flst(k));
        data = csvread(holder);
        
        output_A1 = [output_A1 data(:,target_A1_column)];
        text   = holder(1:end-22);
        name_A1{1,k} = strrep(text,'_',' ');
    end
end

for k = 1 : size(flst,2)  
    if k == 1
        holder = char(flst(k));
        data = csvread(holder);
        
        time_B1   = data(:,t);
        output_B1 = [data(:,target_B1_column)];
        text   = holder(1:end-22);
        name_B1{1,1} = strrep(text,'_',' ');
    else
        holder = char(flst(k));
        data = csvread(holder);
        
        output_B1 = [output_B1 data(:,target_B1_column)];
        text   = holder(1:end-22);
        name_B1{1,k} = strrep(text,'_',' ');
    end
end

h = figure(1);

%%% Graph 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,1,1);
[hA,hA_Line1,hA_Line2] = plotyy(time_A1,output_A1,time_A2,output_A2);
title(target_A1_title);
xlabel('time (s)');

ylabel(hA(1),target_A1_units);
ylabel(hA(2),target_A2_units);
ylim(hA(2),target_A2_limits);
legend(name_A1);

%%% Graph 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,1,2);
[hB,hB_Line1,hB_Line2] = plotyy(time_B1,output_B1,time_B2,output_B2);
title(target_B1_title);
xlabel('time (s)');

ylabel(hB(1),target_B1_units);
ylabel(hB(2),target_B2_units)
ylim(hB(2),target_B2_limits);
legend(name_B1);
