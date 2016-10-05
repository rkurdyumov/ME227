%% Script for 2011 ME227 HW 4 Problem 3
% Author: Ruslan Kurdyumov
% Date: April 25, 2011

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
%hw4_prob2; % Get the parameters for the equilibrium pt
close all 

% Define the simulation parameters for your first simulation. 
% Our state variables are x = [Uy r]' where Uy = lateral velocity, r = yaw rate
% Define the initial conditions of the simulation


% Define testing scenario 
simulation.speed = 8;	

% Define which models we want to use
driver.deltaf = degtorad(-10);
simulation.vmodel = 'bike';  % vehicle model
simulation.tmodel = 'fiala';   
simulation.g = 9.81;

%% 3(a) Linearization about left-hand drift equilibrium w/ controller
% We use the state space equation in the Phase Plane Analysis notes,
% substituting -K*[delta_beta delta_r]' for delta_delta.  Note that we must
% compensate for the expression being written in terms of delta_beta as
% follows: A = [a b*Ux; c/Ux d] and B*K = [e*Ux f*Ux; g h].  Substituting,
% we get the following eigenvalues

m = vehicle.m; L = vehicle.L; b = vehicle.b; a = vehicle.a; 
g = simulation.g; Izz = vehicle.Izz;

% Calculate the normal forces for each wheel
Fz(lf) = b/L*(m*g)/2;
Fz(rf) = b/L*(m*g)/2;
Fz(lr) = a/L*(m*g)/2;
Fz(rr) = a/L*(m*g)/2;

% Replicate Problem 2 equilibrium pt calculations
Uy = -3.1853;
Ux = 8;
r = 0.6499;
alpha_f = radtodeg((Uy + a*r)/Ux - driver.deltaf);
alpha_r = radtodeg((Uy - b*r)/Ux);
alpha = [alpha_f alpha_f alpha_r alpha_r];
dalpha = 0.0001;
Fy = tireforces(simulation,vehicle,alpha*pi/180,Fz);
Fy2 = tireforces(simulation,vehicle,dalpha+alpha*pi/180,Fz);

C = abs(Fy2 - Fy)/dalpha;
C_af0 = (C(1) + C(2));
C_ar0 = (C(3) + C(4));

C0 = C_af0 + C_ar0;
C1 = a*C_af0 - b*C_ar0;
C2 = a^2*C_af0 + b^2*C_ar0;
v = sqrt(Uy^2 + Ux^2);

e = -1/(m*v)*(C_af0 + C_ar0);
f = -1/(m*v^2)*(a*C_af0 - b*C_ar0) - 1;
g = -1/Izz*(a*C_af0 - b*C_ar0);
h = -1/(Izz*v)*(a^2*C_af0 + b^2*C_ar0);

A = [e f*Ux; g/Ux h];
B = [C_af0/(m*v); a*C_af0/Izz];

KUy = -0.3411;
Kr = 0.8742;
K = [KUy Kr];
BK = B*K;
BKnew = [Ux*BK(1,1) Ux*BK(1,2); BK(2,1) BK(2,2)];
eigvalues_CL = eig(A - BKnew)

%% 3(c) Drift controller in simulation
% See plots below.

driver.mode = 'drifting';
driver.deltaEq = degtorad(-10);
driver.UyEq = Uy;
driver.rEq = r;
driver.KUy = KUy;
driver.Kr = Kr;

t0 = 0; tf = 5; tstep = 0.001;
t = t0:tstep:tf;
% x = [Uy r Ux wlf wrf wlr wrr s e delta_psi]
x = zeros(length(t),2);
delta = zeros(length(t),4);
x(1,1) = -4; % Uy(t=0)
x(1,2) = 0.5; % r(t=0)

for i = 1:length(t)
    delta(i,:) = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta(i,:));
    Fy = tireforces(simulation, vehicle, alpha, Fz);
    dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy);
    x(i+1,:) = x(i,:) + tstep*dxdt;
end

Uy = x(1:end-1,1);
beta = Uy/Ux;
beta_nos = atan(Uy/Ux);
r = x(1:end-1,2);

figure;
plot(t, radtodeg(delta(:,1)), 'k');
title('Steering angle response for controlled left-hand equilibrium');
xlabel('time (s)'); ylabel('angle (deg)');
%%
figure;
plot(t, radtodeg(beta), 'k', t, radtodeg(beta_nos), 'k:');
title('Sideslip angle response for controlled left-hand equilibrium');
xlabel('time (s)'); ylabel('angle (deg)');
legend('\beta = U_y/U_x', '\beta = arctan(U_y/U_x)', ...
    'Location', 'SouthEast');
%%
figure;
plot(t, radtodeg(r), 'k');
title('Yaw rate response for controlled left-hand equilibrium');
xlabel('time (s)'); ylabel('angular speed (deg/s)');



% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw4_prob3.m', options));
%}