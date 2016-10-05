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

%% (7.1) Nonlinear four wheel handling diagram w/ ramp input

simulation.vmodel = 'fourwheel';
x_f = zeros(length(t),2);
delta_f = zeros(length(t),4);
dxdt_f = zeros(length(t),2);
Fx = zeros(1,4);

% Four wheel Euler integration
for i = 1:(length(t)-1)
    delta_f(i,:) = steering(simulation, driver, x_f(i,:), t(i));
    alpha = slips(simulation, vehicle, x_f(i,:), delta_f(i,:));
    Fy = tireforces(simulation, vehicle, alpha, Fz);
    dxdt_f(i,:) = derivs(simulation, vehicle, driver, x_f(i,:), Fy, Fx, delta_f(i,:));
    x_f(i+1,:) = x_f(i,:) + tstep*dxdt_f(i,:);
end

Uydot = dxdt(:,1); r = x(:,2); Ux = simulation.speed;
a_lat = Uydot + r*Ux;
Uydot_f = dxdt_f(:,1); r_f = x_f(:,2);
a_lat_f = Uydot_f + r_f*Ux;
figure; plot(a_lat(1:end-1), delta(1:end-1,lf)*180/pi, 'k', ...
    a_lat_f(1:end-1), delta_f(1:end-1,lf)*180/pi, 'k:');
title('Yaw rate for ramp input constant speed understeer gradient test'); 
xlabel('a_{lat} (m/s^2)'); ylabel('steering angle \delta (deg)');
legend('Bike model', 'Four wheel model');

%% (7.1.1)

Fn = 0:8000;
%vehicle.mu_peak = 1.2 - 2e-5*Fz;
%vehicle.mu_slide = 0.8 - 1.5e-5*Fz;
%vehicle.Ca(lf:rf) = 102980*sin(1.5*atan(2.9e-4 * Fz(lf:rf)));
%vehicle.Ca(lr:rr) = 91399*sin(1.5*atan(2.9e-4 * Fz(lr:rr)));
vehicle.hf = 0.01;
vehicle.hr = 0.15;
vehicle.hcg = 0.5;
vehicle.hprime = vehicle.hcg - (vehicle.b/vehicle.L*vehicle.hf + ...
    vehicle.a/vehicle.L*vehicle.hr); % vertical distance from CG to roll axis
vehicle.msprung = 0.9*vehicle.m;

for i=1:length(Fn)
    mu_peak(i) = 1.2 - 2e-5*Fn(i);
    mu_slide(i) = 0.8 - 1.5e-5*Fn(i);
    Ca(i,lf) = 102980*sin(1.5*atan(2.9e-4 * Fn(i)));
    Ca(i,lr) = 91399*sin(1.5*atan(2.9e-4 * Fn(i)));
end
Ca(:,rf) = Ca(:,lf);
Ca(:,rr) = Ca(:,lr);

figure;
plot(Fn, mu_peak, 'k', Fn, mu_slide, 'k:'); hold on;
plot(Fz(lf), 1.2 - 2e-5*Fz(lf), 'ko', Fz(lr), 1.2 - 2e-5*Fz(lr), 'kd',...
    Fz(lf), 0.8 - 1.5e-5*Fz(lf), 'ko', Fz(lr), 0.8 - 1.5e-5*Fz(lr), 'kd');
hold off;
title('Friction coefficients vs. Normal Force');
xlabel('Normal force (N)'); ylabel('(unitless)');
legend('Static', 'Kinetic', 'Straight Front', 'Straight Rear');

figure;
plot(Fn, Ca(:,lf), 'k', Fn, Ca(:,lr), 'k:'); hold on;
plot(Fz(lf), 102980*sin(1.5*atan(2.9e-4 * Fz(lf))), 'ko', ...
    Fz(lr), 102980*sin(1.5*atan(2.9e-4 * Fz(lr))), 'kd', ...
    Fz(lf), 91399*sin(1.5*atan(2.9e-4 * Fz(lf))), 'ko', ...
    Fz(lr), 91399*sin(1.5*atan(2.9e-4 * Fz(lr))), 'ko');
hold off;
title('Cornering stiffness vs. Normal Force');
xlabel('Normal force (N)'); ylabel('(N/rad)');
legend('Static', 'Kinetic', 'Straight front', 'Straight rear');


%% (7.1.2)
% The first case is more understeering.  This is what we expect since the
% front is more stiff, therefore more weight will transfer in the front,
% leading to limit understeer behavior.  The stabilizer bar has an effect
% at high lateral acceleration.

simulation.vmodel = 'fourwheel';
driver.mode = 'ramp';

% Initialize our time vector and state vector
t0 = 0; tf = 30; tstep = 0.001;
t = t0:tstep:tf;
x = zeros(length(t),2);
delta = zeros(length(t),4);
dxdt = zeros(length(t),2);

Kphi_diff = [90000 50000];
Load = Fz;
% Four wheel Euler integration
for i = 1:(length(t)-1)
    vehicle.mu_peak = 1.2 - 2e-5*Load;
    vehicle.mu_slide = 0.8 - 1.5e-5*Load;
    vehicle.Ca(lf:rf) = 102980*sin(1.5*atan(2.9e-4 * Load(lf:rf)));
    vehicle.Ca(lr:rr) = 91399*sin(1.5*atan(2.9e-4 * Load(lr:rr)));
    delta(i,:) = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta(i,:));
    ay = x(i,2)*simulation.speed;
    Load = normalLoad(vehicle,simulation,ay,Kphi_diff,Fz);
    Fy = tireforces(simulation, vehicle, alpha, Load);
    dxdt(i,:) = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta(i,:));
    x(i+1,:) = x(i,:) + tstep*dxdt(i,:);
end
x_diff = x;

Kphi_same = [70000 70000];
Load = Fz;
% Four wheel Euler integration
for i = 1:(length(t)-1)
    vehicle.mu_peak = 1.2 - 2e-5*Load;
    vehicle.mu_slide = 0.8 - 1.5e-5*Load;
    vehicle.Ca(lf:rf) = 102980*sin(1.5*atan(2.9e-4 * Load(lf:rf)));
    vehicle.Ca(lr:rr) = 91399*sin(1.5*atan(2.9e-4 * Load(lr:rr)));
    delta(i,:) = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta(i,:));
    ay = x(i,2)*simulation.speed;
    Load = normalLoad(vehicle,simulation,ay,Kphi_same,Fz);
    Fy = tireforces(simulation, vehicle, alpha, Load);
    dxdt(i,:) = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta(i,:));
    x(i+1,:) = x(i,:) + tstep*dxdt(i,:);
end
x_same = x;

plot(x_diff(1:end-1,2)*simulation.speed, delta(1:end-1,lf), 'k', ...
    x_same(1:end-1,2)*simulation.speed, delta(1:end-1,lf), 'k:');
title('Constant velocity handing diagram');
xlabel('a_y (m/s^2)'); ylabel('delta (rad)');
legend('K_\phi = [90000 50000] N/rad', 'K_\phi = [70000 70000] N/rad', ...
    'Location', 'SouthEast');

%% (7.1.3)

[~, i_same] = max(x_same);
[~, i_diff] = max(x_diff);

ay_same = x_same(i_same(2),2)*simulation.speed;
ay_diff = x_diff(i_diff(2),2)*simulation.speed;

F_same = normalLoad(vehicle,simulation,ay_same,Kphi_same,Fz);
F_diff = normalLoad(vehicle,simulation,ay_diff,Kphi_diff,Fz);

figure;
plot(Fn, mu_peak, 'k', Fn, mu_slide, 'k:'); hold on;
plot(F_same(lf), 1.2 - 2e-5*F_same(lf), 'ko', F_same(lr), 1.2 - 2e-5*F_same(lr), 'kd',...
    F_diff(lf), 0.8 - 1.5e-5*F_diff(lf), 'k*', F_diff(lr), 0.8 - 1.5e-5*F_diff(lr), 'ks');
hold off;
title('Friction coefficients vs. Normal Force for max a_y');
xlabel('Normal force (N)'); ylabel('(unitless)');
legend('Static', 'Kinetic', ...
    'front (K_\phi same)', 'rear (K_\phi same)',...
    'front (K_\phi diff)', 'rear (K_\phi diff)', ...
    'Location', 'East');

figure;
plot(Fn, Ca(:,lf), 'k', Fn, Ca(:,lr), 'k:'); hold on;
plot(F_same(lf), 102980*sin(1.5*atan(2.9e-4 * F_same(lf))), 'ko', ...
    F_same(lr), 102980*sin(1.5*atan(2.9e-4 * F_same(lr))), 'kd', ...
    F_diff(lf), 91399*sin(1.5*atan(2.9e-4 * F_diff(lf))), 'ko', ...
    F_diff(lr), 91399*sin(1.5*atan(2.9e-4 * F_diff(lr))), 'ko');
hold off;
title('Cornering stiffness vs. Normal Force for max a_y');
xlabel('Normal force (N)'); ylabel('(N/rad)');
legend('Static', 'Kinetic', ...
    'front (K_\phi same)', 'rear (K_\phi same)',...
    'front (K_\phi diff)', 'rear (K_\phi diff)', ...
    'Location', 'East');


% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw7_prob1.m', options));
%}