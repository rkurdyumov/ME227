%% Script for 2011 ME227 HW 3 Verify
% Author: Ruslan Kurdyumov
% Date: April 20, 2011

clc; clear all; close all;

% Use these arbitrary values for your states and inputs 
Ux = 30; % longitudinal speed (m/s) 
Uy = -1; % lateral speed (m/s) 
r = -0.5; % yaw rate (rad/s) 
s = 150; % distance down path (m) 
e = 0.5; % lateral error of CG (m) 
deltapsi = 0.1; % heading error (rad) 
delta_l = 0.05; % left steer angle (rad) 
delta_r = 0.05; % right steer angle (rad) 
delta = delta_l; % average steer angle (rad) 
wlf = 100; % wheel speed - front left (rad/s) 
wrf = 95; % wheel speed - front right (rad/s) 
wlr = 99; % wheel speed - rear left (rad/s) 
wrr = 103; % wheel speed - rear right (rad/s) 
taulf = -10; % wheel torque - front left (Nm) 
taurf = 200; % wheel torque - front right (Nm) 
taulr = 35; % wheel torque - rear left (Nm) 
taurr = -75; % wheel torque - rear right (Nm) 

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

addpath('../Common')

simulation.tmodel = 'fiala';
simulation.vmodel = 'fourwheel';
simulation.speed = 20;
% Define any convenient physical parameters
simulation.g = 9.81;

% Driver
driver.mode = 'path';	% Type of steering (control strategy or maneuver)
driver.Ke = 0.1;        % Initial steering angle
driver.Kpsi = 1.0;      % Final steering angle (rad)

%
vehicleTTS;

% Define the track parameters
load racingLineAndTrack;
trackInfo.c = c;
trackInfo.sLine = sLine;
trackInfo.radius = 95;

Fz(lf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(lr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;

x = [Uy r Ux wlf wrf wlr wrr s e deltapsi];
delta(lf) = delta_l; delta(rf) = delta_r; delta(lr) = 0; delta(rr) = 0;
tau(lf) = taulf; tau(rf) = taurf; tau(lr) = taulr; tau(rr) = taurr;
[alpha, kappa] = slips(simulation, vehicle, x, delta)
[Fy, Fx] = tireforces(simulation, vehicle, alpha, Fz, kappa)
dxdt = derivs(simulation, vehicle, driver, x, Fy, Fx, delta, trackInfo, tau);
dxdt'

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('verify.m', options));
%}

%% Check your results against these output values

% alpha =
% 
%    -0.0996
%    -0.1009
%    -0.0091
%    -0.0093
% 
% 
% kappa =
% 
%     0.1343
%     0.1059
%     0.1187
%     0.1945
% 
% 
% Fy =
% 
%   1.0e+003 *
% 
%     1.1796
%     1.4587
%     0.0724
%     0.0454
% 
% 
% Fx =
% 
%   1.0e+003 *
% 
%     3.1693
%     3.0510
%     2.3733
%     2.3740
% 
% 
% dxdt =
% 
%    17.0885
%     1.2198
%     7.8815
%  -915.2832
%  -706.4149
%  -649.9912
%  -741.8495
%    29.9500
%     2.0000
%    -0.6454