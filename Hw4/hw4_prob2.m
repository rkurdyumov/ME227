%% Script for 2011 ME227 HW 4 Problem 2
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

% Define the simulation parameters for your first simulation. 
% Our state variables are x = [Uy r]' where Uy = lateral velocity, r = yaw rate
% Define the initial conditions of the simulation


% Define testing scenario 
driver.mode = 'step';	% Type of steering (control strategy or maneuver)
driver.delta0 = 0;      % Initial steering angle
driver.deltaf =	-10*pi/180; % Final steering angle (rad)
driver.steertime = 0;	% Time to start the step
simulation.speed = 8;	

% Define which models we want to use
simulation.vmodel = 'bike';  % vehicle model
simulation.tmodel = 'fiala';   

%% 2(a) Tire curves: Fiala bike model
% The results are below, given as [Focus Saddle1(r>0) Saddle2(r<0)]
%
% The plots show that the stable focus has front and rear forces in the
% diretion of delta, which makes sense.  We are also not in the saturated
% regions of the tire force curve.  Saddle1 is clearly unstable - the rear
% tire is heavily saturated, so we are limit oversteer. Saddle2 looks OK
% in the front, but the rear tire is also saturated, leading to a limit
% oversteer, also unstable.
% 
% Saddle point 1 (r>0)  corresponds to a left-hand drift.  Our yaw rate is
% counterclockwise, and the rear tire is competely saturated, leading to
% severe slip, therefore "turning right to go left".

% Define any convenient physical parameters
simulation.g = 9.81;
g = simulation.g;

m = vehicle.m; L = vehicle.L; b = vehicle.b; a = vehicle.a; Izz = vehicle.Izz;

% Calculate the normal forces for each wheel
Fz(lf) = b/L*(m*g)/2;
Fz(rf) = b/L*(m*g)/2;
Fz(lr) = a/L*(m*g)/2;
Fz(rr) = a/L*(m*g)/2;

% Simulate the linear and fiala tire models
angle = -30:0.05:30;
for i=1:length(angle)
    simulation.tmodel = 'fiala';
    FyNL(i,:) = tireforces(simulation,vehicle,angle(i)*pi/180.*ones(1,4),Fz);    
end

% Plot the tire curves
figure; plot(angle, FyNL(:,1) + FyNL(:,2), 'k');
hold on;
% Add the equilibrium points to our plot


Uy = [-0.2425 -3.1853 0.3928];
Ux = 8;
r = [-0.4942 0.6499 -0.6499];

beta = radtodeg(Uy/Ux)
alpha_f = radtodeg((Uy + a*r)/Ux - driver.deltaf)
alpha_r = radtodeg((Uy - b*r)/Ux)

for i=1:3
    Fyf(i) = 2*interp1q(angle, FyNL(:,1), alpha_f(i));
end
plot(alpha_f(1), Fyf(1), 'k*', alpha_f(2), Fyf(2), 'kd', alpha_f(3), Fyf(3), 'ko');

plot(angle, FyNL(:,3) + FyNL(:,4), 'k-.');
% Add the equilibrium points to our plot
for i=1:3
    Fyr(i) = 2*interp1q(angle, FyNL(:,3), alpha_r(i));
end
plot(alpha_r(1), Fyr(1), 'k*', alpha_r(2), Fyr(2), 'kd', alpha_r(3), Fyr(3), 'ko');
hold off;
title('Total lateral force tire curve');
xlabel('angle (deg)'); ylabel('force (N)');
legend('Front tire curve', 'Focus ', 'Saddle1 (r>0)', ...
    'Saddle2 (r<0)', 'Rear tire curve');

%% 2(b) Effective cornering stiffness
% See below.

% Create vector of slip angles for each equilibrium
alpha = [];
for i=1:3
    alpha = [alpha; alpha_f(i) alpha_f(i) alpha_r(i) alpha_r(i)];
end

% Find the change in lateral force for a small change in slip angle
dalpha = 0.0001;
for i = 1:3
    Fy(i,:) = tireforces(simulation,vehicle,alpha(i,:)*pi/180,Fz);
    Fy2(i,:) = tireforces(simulation,vehicle,dalpha+alpha(i,:)*pi/180,Fz);
end

C = abs(Fy2 - Fy)/dalpha;
C_af0 = (C(:,1) + C(:,2))'
C_ar0 = (C(:,3) + C(:,4))'

%% 2(b) Local understeer gradient
% See below.  The eigenvalues are close to what PPlane gives, which is what
% we expect, but vary slightly since we did not assume that v = Ux when
% solving for the roots of the characteristic equation.

for i=1:3
    K(i) = m/L*(b/C_af0(i) - a/C_ar0(i)) * g;
end
K

% Calculate eigenvalues
C0 = C_af0 + C_ar0;
C1 = a*C_af0 - b*C_ar0;
C2 = a^2*C_af0 + b^2*C_ar0;
v = sqrt(Uy.^2 + Ux^2);

for i=1:3
    eigvalues(:,i) = roots([Izz, (C0(i)*Izz + m*C2(i))/(m*v(i)), ...
        C0(i)*C2(i)/(m*v(i)^2) - C1(i) - C1(i)^2/(m*v(i)^2)]);
end
eigvalues

%% 2(b) Yaw rate step response to small steering perturbation
% Only our stable equilibrium has a stable step response, as expected.

label = {' Stable focus', ' Saddle1 (r>0)', ' Saddle2 (r<0)'};
% delr/deldelta
hold on;
for i=1:3
    figure; 
    num = [a*C_af0(i), (a*C_af0(i)*C0(i) - C1(i)*C_af0(i))/(m*v(i))];
    den = [Izz, (C0(i)*Izz + m*C2(i))/(m*v(i)), ...
        C0(i)*C2(i)/(m*v(i)^2) - C1(i) - C1(i)^2/(m*v(i)^2)];
    sys = tf(num,den);
    step(degtorad(0.1)*sys, 'k');
    title(['\Deltar(s)/\Delta\delta(s) 0.1 deg step response for', label{i}]);
    xlabel('time (s)'); ylabel('rad/s');
end

%% 2(c) Uy step response to small steering perturbation
% Only our stable equilibrium has a stable step response, as expected.

for i=1:3
    figure;
    num = Ux*[C_af0(i)/(m*v(i))*Izz, ...
        (C2(i)*C_af0(i) - a*C1(i)*C_af0(i))/(m*v(i)^2) - a*C_af0(i)];
    den = [Izz, (C0(i)*Izz + m*C2(i))/(m*v(i)), ...
        C0(i)*C2(i)/(m*v(i)^2) - C1(i) - C1(i)^2/(m*v(i)^2)];
    sys = tf(num,den);
    step(degtorad(0.1)*sys, 'k');
    title(['\DeltaU_y(s)/\Delta\delta(s) 0.1 deg step response for', label{i}]);
    xlabel('time (s)'); ylabel('rad/s');
end

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw4_prob2.m', options));
%}