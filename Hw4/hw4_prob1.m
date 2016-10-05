%% Script for 2011 ME227 HW 4 Problem 1
% Author: Ruslan Kurdyumov
% Date: April 17, 2011

% Make sure you have a clean environment in which to work
clear all
close all 
clc

% Add the path to our common files that we will often recycle
addpath('../Common')

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% First load the parameters for the vehicle you are going to use
vehicleP1;
vehicle = P1;

% Define any convenient physical parameters
simulation.g = 9.81;

%% 1(a) Linear bike model
% We expect to see one equilibrium for the linear model.  See the
% understeer gradient below.  Since it is positive, we expect an
% understeering car.

m = vehicle.m; L = vehicle.L; b = vehicle.b; a = vehicle.a; 
g = simulation.g; Izz = vehicle.Izz;

Ca = vehicle.Ca;

% Calculate the normal forces for each wheel
Fz(lf) = b/L*(m*g)/2;
Fz(rf) = b/L*(m*g)/2;
Fz(lr) = a/L*(m*g)/2;
Fz(rr) = a/L*(m*g)/2;

Caf = Ca(lf) + Ca(rf);
Car = Ca(lr) + Ca(rr);

K = m*g/L*(b/Caf - a/Car)

%% 1(b) Phase portrait: linear bike model
% We find one equilibrium: a stable node at (Uy = -0.3617, r = -0.5104).

imshow('linear.bmp', 'Border', 'tight')

%% 1(c) Phase portrait: nonlinear bike model
% We find three equilibria:
% (1) stable focus at (Uy = -0.2425, r = -0.4942).
% (2) unstable saddlepoint at (Uy = -3.1853, r = 0.6499)
% (3) unstable saddlepoint at (Uy = 0.3928, r = -0.6499)
% All of our equilibria have complex eigenvalues.  We still have one stable
% equilibrium, a node in the linear case (with negative eigenvalues), and a
% focus in the nonlinear case (with LHP complex eigenvalues).

imshow('nonlinear.bmp', 'Border', 'tight')

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw4_prob1.m', options));
%}
