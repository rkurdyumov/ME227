%% Script for 2011 ME227 HW 2 Problem 2
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

% Define testing scenario 
driver.mode = 'ramp';	% Type of steering (control strategy or maneuver)
driver.delta0 = 0;      % Initial steering angle
driver.deltaramp = 0.01; % Ramp increase in steering angle (rad/s) 
driver.steertime = 0;	% Time to start the ramp
simulation.speed = 10;	% Perform the maneuver at 10 m/s
simulation.tmodel = 'fiala';
simulation.vmodel = 'bike';

% Define any convenient physical parameters
simulation.g = 9.81;

%% (2.4) Nonlinear bike handling diagram w/ ramp input

% Calculate the normal forces for each wheel
Fz(lf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(lr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;

% Initialize our time vector and state vector
t0 = 0; tf = 40; tstep = 0.001;
t = t0:tstep:tf;
x = zeros(length(t),2);
delta = zeros(length(t),4);
dxdt = zeros(length(t),2);

% Bike Euler integration
for i = 1:(length(t)-1)
    delta(i,:) = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta(i,:));
    Fy = tireforces(simulation, vehicle, alpha, Fz);
    dxdt(i,:) = derivs(simulation, vehicle, driver, x(i,:), Fy);
    x(i+1,:) = x(i,:) + tstep*dxdt(i,:);
end

%% Plot
Uydot = dxdt(:,1); r = x(:,2); Ux = simulation.speed;
a_lat = Uydot + r*Ux;
figure; plot(a_lat(1:end-1), delta(1:end-1,lf)*180/pi, 'k');
title('Yaw rate for ramp input constant speed understeer gradient test'); 
xlabel('a_{lat} (m/s^2)'); ylabel('steering angle \delta (deg)');


% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw2_prob2.m', options));
%}