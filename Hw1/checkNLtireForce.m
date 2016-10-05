% Script for 2011 ME227 Checking NL tire force
% Author: Mick
% Date: April 5, 11

% Make sure you have a clean environment in which to work
clear all
close all 
clc

% Add the path to our common files that we will often recycle
addpath('../Common')

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% First load the parameters for the vehicle you are going to use
vehicleTTS

% Define which models we want to use
simulation.vmodel = 'bike';  % vehicle model
simulation.speed = 20;	% Perform the maneuver at 20 m/s
simulation.tmodel = 'fiala';    % nonlinear tire model, or Fiala tire model, if this is what you use

% Define any convenient physical parameters
simulation.g = 9.81;

% Calculate the normal forces for each wheel
% put your script in here
Fz(lf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(lr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;
                        
FyNL=tireforces(simulation,vehicle,1*pi/180.*ones(1,4),Fz)
FyNL=tireforces(simulation,vehicle,2*pi/180.*ones(1,4),Fz)
FyNL=tireforces(simulation,vehicle,3*pi/180.*ones(1,4),Fz)




