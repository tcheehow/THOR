function h_1 = main_presenter(t,x)

% Linear Position
h_1(1) = figure(1);
subplot(3,1,1);
plot(t,x(:,1));
title('P_x');
xlabel('time (s)');
ylabel('(m)');

subplot(3,1,2);
plot(t,x(:,2));
title('P_y');
xlabel('time (s)');
ylabel('(m)');

subplot(3,1,3);
plot(t,x(:,3));
title('P_z');
xlabel('time (s)');
ylabel('(m)');

% Angular Position
h_1(2) = figure(2);
subplot(3,1,1);
plot(t,x(:,4));
title('theta_x');
xlabel('time (s)');
ylabel('(rad)');

subplot(3,1,2);
plot(t,x(:,5));
title('theta_y');
xlabel('time (s)');
ylabel('(rad)');

subplot(3,1,3);
plot(t,x(:,6));
title('theta_z');
xlabel('time (s)');
ylabel('(rad)');

% Linear Velocity
h_1(3) = figure(3);
subplot(3,1,1);
plot(t,x(:,7));
title('v_x');
xlabel('time (s)');
ylabel('(m/s)');

subplot(3,1,2);
plot(t,x(:,8));
title('v_y');
xlabel('time (s)');
ylabel('(m/s)');

subplot(3,1,3);
plot(t,x(:,9));
title('v_z');
xlabel('time (s)');
ylabel('(m/s)');

% Angular Velocity
h_1(4) = figure(4);
subplot(3,1,1);
plot(t,x(:,10));
title('omega_x');
xlabel('time (s)');
ylabel('(rad/s)');

subplot(3,1,2);
plot(t,x(:,11));
title('omega_y');
xlabel('time (s)');
ylabel('(rad/s)');

subplot(3,1,3);
plot(t,x(:,12));
title('omega_z');
xlabel('time (s)');
ylabel('(rad/s)');
