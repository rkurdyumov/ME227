%% Script for 2011 ME227 HW 7 Problem 1
% Author: Ruslan Kurdyumov
% Date: May 19, 2011

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

% Define testing scenario 
driver.mode = 'ramp';	% Type of steering (control strategy or maneuver)
driver.delta0 = 0;      % Initial steering angle
driver.deltaramp = 0.01; % Ramp increase in steering angle (rad/s) 
driver.steertime = 0;	% Time to start the ramp
simulation.speed = 10;	% Perform the maneuver at 10 m/s
simulation.tmodel = 'fiala';
simulation.vmodel = 'fourwheel';

% Define any convenient physical parameters
simulation.g = 9.81;

vehicle.hf = 0.01;
vehicle.hr = 0.15;
vehicle.hcg = 0.5;
vehicle.hprime = vehicle.hcg - (vehicle.b/vehicle.L*vehicle.hf + ...
    vehicle.a/vehicle.L*vehicle.hr); % vertical distance from CG to roll axis
vehicle.msprung = 0.9*vehicle.m;

b = vehicle.b;
a = vehicle.a;
L = vehicle.L;
m = vehicle.m;
g = simulation.g;

%% (7.2.1): 60% cross weight

xw = 0.50;
A = [1 1 0 0; 0 0 1 1; 0 1 1 0; 1 1 1 1];
F = m*g*[b/L; a/L; xw; 1];
Fz_50 = pinv(A)*F

xw = 0.60;
A = [1 1 0 0; 0 0 1 1; 0 1 1 0; 1 1 1 1];
F = m*g*[b/L; a/L; xw; 1];
Fz_60 = pinv(A)*F

%% (7.2.2): Handling diagram: 60% vs. 50% cross weight
% Changing the cross weight to 60% makes the vehicle more understeering
% when doing a left-hand turn.

% Initialize our time vector and state vector
t0 = 0; tf = 30; tstep = 0.001;
t = t0:tstep:tf;
x = zeros(length(t),2);
delta = zeros(length(t),4);
dxdt = zeros(length(t),2);
Fx = zeros(1,4);

Kphi = [90000 50000];
Load = Fz_60;
% Four wheel Euler integration
for i = 1:(length(t)-1)
    vehicle.mu_peak = 1.2 - 2e-5*Load;
    vehicle.mu_slide = 0.8 - 1.5e-5*Load;
    vehicle.Ca(lf:rf) = 102980*sin(1.5*atan(2.9e-4 * Load(lf:rf)));
    vehicle.Ca(lr:rr) = 91399*sin(1.5*atan(2.9e-4 * Load(lr:rr)));
    delta(i,:) = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta(i,:));
    ay = x(i,2)*simulation.speed;
    Load = normalLoad(vehicle,simulation,ay,Kphi,Fz_60);
    Fy = tireforces(simulation, vehicle, alpha, Load);
    dxdt(i,:) = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta(i,:));
    x(i+1,:) = x(i,:) + tstep*dxdt(i,:);
end
x_60 = x;

Load = Fz_50;
% Four wheel Euler integration
for i = 1:(length(t)-1)
    vehicle.mu_peak = 1.2 - 2e-5*Load;
    vehicle.mu_slide = 0.8 - 1.5e-5*Load;
    vehicle.Ca(lf:rf) = 102980*sin(1.5*atan(2.9e-4 * Load(lf:rf)));
    vehicle.Ca(lr:rr) = 91399*sin(1.5*atan(2.9e-4 * Load(lr:rr)));
    delta(i,:) = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta(i,:));
    ay = x(i,2)*simulation.speed;
    Load = normalLoad(vehicle,simulation,ay,Kphi,Fz_50);
    Fy = tireforces(simulation, vehicle, alpha, Load);
    dxdt(i,:) = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta(i,:));
    x(i+1,:) = x(i,:) + tstep*dxdt(i,:);
end
x_50 = x;

plot(x_60(1:end-1,2)*simulation.speed, delta(1:end-1,lf), 'k', ...
    x_50(1:end-1,2)*simulation.speed, delta(1:end-1,lf), 'k:');
title('Constant velocity handing diagram (K_\phi = [90000 50000]');
xlabel('a_y (m/s^2)'); ylabel('delta (rad)');
legend('60% cross weight', '50% cross weight', 'Location', 'SouthEast');

%% (7.2.3): 40% cross weight
% See the peak lateral accelerations below.  Decreasing the cross weight
% causes the peak lateral acceleration to increase, so the 40% cross weight
% produces the highest peak lateral acceleration.  This makes sense because
% increasing the cross weight increases the load on the front outside tire
% and decreases the load on the rear outside tire, so we will saturate the
% front first, and therefore not reach as high of a lateral acceleration.

xw = 0.40;
A = [1 1 0 0; 0 0 1 1; 0 1 1 0; 1 1 1 1];
F = m*g*[b/L; a/L; xw; 1];
Fz_40 = pinv(A)*F

Load = Fz_40;
% Four wheel Euler integration
for i = 1:(length(t)-1)
    vehicle.mu_peak = 1.2 - 2e-5*Load;
    vehicle.mu_slide = 0.8 - 1.5e-5*Load;
    vehicle.Ca(lf:rf) = 102980*sin(1.5*atan(2.9e-4 * Load(lf:rf)));
    vehicle.Ca(lr:rr) = 91399*sin(1.5*atan(2.9e-4 * Load(lr:rr)));
    delta(i,:) = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta(i,:));
    ay = x(i,2)*simulation.speed;
    Load = normalLoad(vehicle,simulation,ay,Kphi,Fz_40);
    Fy = tireforces(simulation, vehicle, alpha, Load);
    dxdt(i,:) = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta(i,:));
    x(i+1,:) = x(i,:) + tstep*dxdt(i,:);
end
x_40 = x;

plot(x_60(1:end-1,2)*simulation.speed, delta(1:end-1,lf), 'k', ...
    x_50(1:end-1,2)*simulation.speed, delta(1:end-1,lf), 'k:', ...
    x_40(1:end-1,2)*simulation.speed, delta(1:end-1,lf), 'k--');
title('Constant velocity handing diagram (K_\phi = [90000 50000]');
xlabel('a_y (m/s^2)'); ylabel('delta (rad)');
legend('60% cross weight', '50% cross weight', '40% cross weight',...
    'Location', 'SouthEast');

[~, i_40] = max(x_40);
[~, i_50] = max(x_50);
[~, i_60] = max(x_60);

ay_max_40 = x_40(i_40(2),2)*simulation.speed
ay_max_50 = x_50(i_50(2),2)*simulation.speed
ay_max_60 = x_60(i_60(2),2)*simulation.speed

%% (7.2.4): 40% cross weight: right hand turn
% It will behave similarly to the 60% cross weight left hand turn since
% these cases are symmetric.

%% (7.2.5): Limit oversteer compensation
% To minimize the limit oversteer, we should increase the cross weight
% since this makes our vehicle more understeering.

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw7_prob2.m', options));
%}