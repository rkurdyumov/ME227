%% Script for 2011 ME227 HW 1 Problem 3
% Author: Ruslan Kurdyumov
% Date: April 6, 2011

% Make sure you have a clean environment in which to work
clear all
close all 
clc

% Add the path to our common files that we will often recycle
addpath('../Common')

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% First load the parameters for the vehicle you are going to use
vehicleTTS;

% Define the simulation parameters for your first simulation. 
% Our state variables are x = [Uy r]' where Uy = lateral velocity, r = yaw rate
% Define the initial conditions of the simulation

% Load our data
load kkritayakirana

% Define testing scenario 
simulation.tmodel = 'linear'; 
driver.mode = 'data';	% Type of steering (control strategy or maneuver)
simulation.vmodel = 'bike';  % vehicle model

% Define any convenient physical parameters
simulation.g = 9.81;

% Find the periods of zero speed and trim all vectors
I = find(abs(Speed_mps) < 1);
Speed_mps(I) = [];
time_s(I) = [];
BrakePressure_bar(I) = [];
LateralAcceleration_mps2(I) = [];
Sideslip_rad(I) = [];
ThrottlePosition_percent(I) = [];
YawRate_radps(I) = [];
deltaRoadwheels_rad(I) = [];

% Update our driver parameters
driver.t_exp = time_s;  % Experimental time vector
driver.delta_exp = deltaRoadwheels_rad; % Experimental delta vector

% Find the index of the last cut zero speed point (used for reading 
% from the delta vector in steering.m)
last_cut = find(I < size(Speed_mps(:,1), 1)/2, 1, 'last' );
driver.last_cut = last_cut; % Update for access in steering.m

%% (3.1) Check steering function

deltaL = zeros(size(time_s,1),4);
for i = 1:size(time_s,1)
    deltaL(i,:) = steering(simulation, driver, [0 0], time_s(i));
end

disp('The experimental and interpolated plots match:');
figure; plot(time_s, deltaRoadwheels_rad, 'k', time_s, deltaL(:,1), 'k:');
title('Steer data from Lab 1'); xlabel('time (s)'); ylabel('\delta (rad)');
legend('Experimental', 'Interpolated output', 'Location', 'SouthWest');

%% (3.2) Experimental vs. linear model yaw rate and side slip (flat section) 

disp('The experimental and linear results match fairly closely');
disp(' ');

figure; plot(Speed_mps, 'k');
title('Speed data from Lab 1'); xlabel('sample'); ylabel('speed (m/s)');

% Choose samples with a relatively constant speed
s_start = 3500; s_stop = 10000;
avg_speed = mean(Speed_mps(s_start:s_stop));
simulation.speed = avg_speed;
disp(['We chose to use data from samples ', num2str(s_start), ' to ', ...
   num2str(s_stop), ' with an average speed ', num2str(avg_speed), 'm/s']);

% Initialize our time vector and state vector
simulation.tmodel = 'linear'; 
t0 = time_s(s_start); 
tf = time_s(s_stop); 
tstep = time_s(s_start) - time_s(s_start-1);
t = t0:tstep:tf+tstep;
x = zeros(round((tf - t0) / tstep + 1),2);
x(1,1) = tan(Sideslip_rad(s_start))*Speed_mps(s_start);
x(1,2) = YawRate_radps(s_start);

% Euler integration
for i = 1:(size(t,2)-1)
    delta = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta);
    Fy = tireforces(simulation, vehicle, alpha);
    dxdt = derivs(simulation, vehicle, x(i,:), Fy);
    x(i+1,:) = x(i,:) + tstep*dxdt;
end

r_L = 180/pi*x(:,2);
r_exp = 180/pi*YawRate_radps(s_start:s_stop);
figure; plot(time_s(s_start:s_stop), r_exp, 'k', t, r_L, 'k:');
title(['Yaw rate from Lab 1 (U_x = ', num2str(avg_speed), 'm/s)']); 
xlabel('time (s)'); ylabel('yaw rate (deg/s)');
legend('Experimental', 'Linear model', 'Location', 'SouthWest');

Uy = x(:,1);
Ux = avg_speed;
beta_L = 180/pi*atan(Uy./Ux);
beta_exp = 180/pi*Sideslip_rad(s_start:s_stop);
figure; plot(time_s(s_start:s_stop), beta_exp, 'k', t, beta_L, 'k:');
title(['Sideslip angle from Lab 1 (U_x = ', num2str(avg_speed), 'm/s)']); 
xlabel('time (s)'); ylabel('\beta (deg)');
legend('Experimental', 'Linear model', 'Location', 'SouthWest');

%% (3.2) Experimental vs. linear model yaw rate and side slip (entire test) 

disp('The experimental and linear results match, but we have more discrepancy:');

% Initialize our time vector and state vector
simulation.tmodel = 'linear'; 
t = time_s;
x = zeros(size(t,1),2);
x(1,1) = tan(Sideslip_rad(1))*Speed_mps(1); % tan(beta) = Uy/Ux
x(1,2) = YawRate_radps(1);

% Euler integration
for i = 1:(size(t,1)-1)
    delta = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta);
    Fy = tireforces(simulation, vehicle, alpha);
    dxdt = derivs(simulation, vehicle, x(i,:), Fy);
    x(i+1,:) = x(i,:) + tstep*dxdt;
end

r_L = 180/pi*x(:,2);
r_exp = 180/pi*YawRate_radps;
figure; plot(time_s, r_exp, 'k', t, r_L, 'k:');
title(['Full test yaw rate from Lab 1 (U_x = ', num2str(avg_speed), 'm/s)']); 
xlabel('time (s)'); ylabel('yaw rate (deg/s)');
legend('Experimental', 'Linear model', 'Location', 'SouthWest');

Uy = x(:,1);
Ux = avg_speed;
beta_L = 180/pi*atan(Uy./Ux); % tan(beta) = Uy/Ux
beta_exp = 180/pi*Sideslip_rad;
figure; plot(time_s, beta_exp, 'k', t, beta_L, 'k:');
title(['Full test sideslip angle from Lab 1 (U_x = ', num2str(avg_speed), 'm/s)']); 
xlabel('time (s)'); ylabel('\beta (deg)');
legend('Experimental', 'Linear model', 'Location', 'SouthWest');

%% (3.3) Experimental vs. linear model yaw rate and side slip (vary Ux) 

disp('It is important to consider the speed variation, since the model');
disp('is much more predictive when we consider it:');

% Initialize our time vector and state vector
simulation.tmodel = 'linear';
driver.mode = 'data';
driver.t_exp = time_s;
driver.delta_exp = deltaRoadwheels_rad;
t = time_s;
tstep = 0.005;
x = zeros(size(t,1),2);
x(1,1) = tan(Sideslip_rad(1))*Speed_mps(1); % tan(beta) = Uy/Ux
x(1,2) = YawRate_radps(1);

% Euler integration
for i = 1:(size(t,1)-1)
    simulation.speed = Speed_mps(i);
    delta = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta);
    Fy = tireforces(simulation, vehicle, alpha);
    dxdt = derivs(simulation, vehicle, x(i,:), Fy);
    x(i+1,:) = x(i,:) + tstep*dxdt;
end

% Plot
r_L = 180/pi*x(:,2);
r_exp = 180/pi*YawRate_radps;
figure; plot(time_s, r_exp, 'k', t, r_L, 'k:');
title('Full test yaw rate from Lab 1 (U_x updated at each step)'); 
xlabel('time (s)'); ylabel('yaw rate (deg/s)');
legend('Experimental', 'Linear model', 'Location', 'SouthWest');

Uy = x(:,1);
beta_L = 180/pi*atan(Uy./Speed_mps);
beta_exp = 180/pi*Sideslip_rad;
figure; plot(time_s, beta_exp, 'k', t, beta_L, 'k:');
title('Full test sideslip angle from Lab 1 (U_x updated at each step)'); 
xlabel('time (s)'); ylabel('\beta (deg)');
legend('Experimental', 'Linear model', 'Location', 'SouthWest');


% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw1_prob3.m', options));
%}