%% Script for 2011 ME227 HW 2 Problem 1
% Author: Ruslan Kurdyumov
% Date: April 11, 2011

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
%load kkritayakirana
load ruslankurdyumov

% Define testing scenario 
driver.mode = 'data';	% Type of steering (control strategy or maneuver)
simulation.tmodel = 'linear';

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


%% (1.3) Experimental vs. bike vs. four wheel model yaw rate & side slip (vary Ux) 
% No, it seems like the two wheel assumptions of the bicycle model don't
% introduce much more error than the full nonlinear four wheel model.
% During sharp turns, there is some deviation, but both the four wheel and
% the bike model deviate from the experimental data moreso than they
% deviate from each other.

% Initialize our time vector and state vector
driver.t_exp = time_s;
driver.delta_exp = deltaRoadwheels_rad;
t = time_s;
tstep = 0.005;
x = zeros(size(t,1),2);
x(1,1) = tan(Sideslip_rad(1))*Speed_mps(1); % Uy = tan(beta)*Ux
x(1,2) = YawRate_radps(1);

simulation.vmodel = 'bike'; 
% Bike Euler integration
for i = 1:(size(t,1)-1)
    simulation.speed = interp1q(driver.t_exp, Speed_mps, t(i));
    delta = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta);
    Fy = tireforces(simulation, vehicle, alpha);
    dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy);
    x(i+1,:) = x(i,:) + tstep*dxdt;
end
x_bike = x;

simulation.vmodel = 'fourwheel';
x = zeros(size(t,1),2);
x(1,1) = tan(Sideslip_rad(1))*Speed_mps(1); % Uy = tan(beta)*Ux
x(1,2) = YawRate_radps(1);
Fx = zeros(1,4);
% Four wheel Euler integration
for i = 1:(size(t,1)-1)
    simulation.speed = interp1q(driver.t_exp, Speed_mps, t(i));
    delta = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta);
    Fy = tireforces(simulation, vehicle, alpha);
    dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta);
    x(i+1,:) = x(i,:) + tstep*dxdt;
end
x_fourw = x;

% Plot
r_bike = 180/pi*x_bike(:,2); 
r_fourw = 180/pi*x_fourw(:,2);
r_exp = 180/pi*YawRate_radps;
figure; plot(time_s, r_exp, 'k', t, r_bike, 'k:', t, r_fourw, 'k--');
title('Yaw rate from Lab 1 (U_x updated at each step)'); 
xlabel('time (s)'); ylabel('yaw rate (deg/s)');
legend('Experimental', 'Linear bike model', 'Four wheel model', ...
'Location', 'SouthWest');

ti = 3300; tf = 3600;
figure; plot(time_s(ti:tf), r_exp(ti:tf), 'k', t(ti:tf), r_bike(ti:tf), 'k:', ...
    t(ti:tf), r_fourw(ti:tf), 'k--');
title('Yaw rate from Lab 1 [zoomed in]'); 
xlabel('time (s)'); ylabel('yaw rate (deg/s)');
legend('Experimental', 'Linear bike model', 'Four wheel model', ...
'Location', 'SouthWest');

Uy_bike = x_bike(:,1); Uy_fourw = x_fourw(:,1);
beta_bike = 180/pi*atan(Uy_bike./Speed_mps);
beta_fourw = 180/pi*atan(Uy_fourw./Speed_mps);
beta_exp = 180/pi*Sideslip_rad;
figure; plot(time_s, beta_exp, 'k', t, beta_bike, 'k:', t, beta_fourw, 'k--');
title('Full test sideslip angle from Lab 1 (U_x updated at each step)'); 
xlabel('time (s)'); ylabel('\beta (deg)');
legend('Experimental', 'Linear bike model', 'Four wheel model', ...
'Location', 'SouthWest');

figure; plot(time_s(ti:tf), beta_exp(ti:tf), 'k', ... 
    t(ti:tf), beta_bike(ti:tf), 'k:', t(ti:tf), beta_fourw(ti:tf), 'k--');
title('Full test sideslip angle from Lab 1 [zoomed in]'); 
xlabel('time (s)'); ylabel('\beta (deg)');
legend('Experimental', 'Linear bike model', 'Four wheel model', ...
'Location', 'SouthWest');

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw2_prob1.m', options));
%}