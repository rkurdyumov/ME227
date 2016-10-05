%% Script for 2011 ME227 HW 3 Problem 2
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
vehicleTTS;

% Define the simulation parameters for your first simulation. 
% Our state variables are x = [Uy r]' where Uy = lateral velocity, r = yaw rate
% Define the initial conditions of the simulation

% Define testing scenario 
driver.mode = 'path';	% Type of steering (control strategy or maneuver)
driver.Ke = 0.1;        % Initial steering angle
driver.Kpsi = 1.0;      % Final steering angle (rad)
simulation.tmodel = 'fiala';
simulation.vmodel = 'fourwheel'; 

% Define any convenient physical parameters
simulation.g = 9.81;

% Define the track parameters
load racingLineAndTrack;
trackInfo.c = c;
trackInfo.sLine = sLine;
trackInfo.radius = 95;

% Calculate the normal forces for each wheel
Fz(lf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(lr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;

%% 2(a) Lateral error vs. Distance along path
% The cornering speed that generates 0.8g lateral acceleration is
% sqrt(R*0.8g).  The lateral error doesn't go to zero because our control
% law isn't as effective away from the linear region (when we corner at
% 0.8g).  During cornering, nonlinear tire force effects invalidate the 
% linear system assumptions of our control law.

simulation.speed = sqrt(trackInfo.radius*0.8*simulation.g)
Ux = simulation.speed;

speed = [Ux, Ux+0.5, Ux+1.0, Ux+1.5];
t_sim = zeros(length(speed),1);
marker = {'k', 'k:', 'k--', 'k-.'};

figure; hold on;
for k = 1:length(speed);
    
    % Initialize state
    t0 = 0; tf = 30; tstep = 0.001;
    t = t0:tstep:tf;
    % x = [Uy r Ux wlf wrf wlr wrr s e delta_psi]
    x = zeros(length(t),10);

    x(:,3) = speed(k);
    x(1,8) = 0; % s(t=0)
    x(1,9) = 1; % e(t=0)
    Fx = zeros(1,4);

    % Euler integration
    i = 1;
    while x(i,8) < trackInfo.sLine(6)
        delta = steering(simulation, driver, x(i,:), t(i));
        [alpha, kappa] = slips(simulation, vehicle, x(i,:), delta);
        Fy = tireforces(simulation, vehicle, alpha, Fz);
        dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo);
        dxdt(3) = 0;
        x(i+1,:) = x(i,:) + tstep*dxdt;
        i = i + 1;
    end

    t_sim(k) = t(i);
    x(i+1:end,:) = [];
    e = x(:,9);
    s = x(:,8);
    plot(s, e, marker{k});
end
hold off;
title('Lateral error vs. distance along track');
xlabel('Distance (m)'); ylabel('Lateral error (m)');
legend(['Ux = ', num2str(Ux), 'm/s [= sqrt(R*0.8g)]'], ...
    ['Ux = ', num2str(Ux+0.5), 'm/s'], ['Ux = ', num2str(Ux+1.0), 'm/s'], ...
    ['Ux = ', num2str(Ux+1.5), 'm/s'], 'Location', 'North');

%% 2(b) Lateral error vs. Distance along path (e > 3)
% In our last simulation, the car goes off the path and our control law is
% no longer able to stabilize it.  As speed increases, our previous plot
% showed that the lateral error increased as well.  Our total time to run
% the track decreased as speed increased as long as we stayed on the track.

% Simulate the speed that causes |e| > 3
speed(end+1) = Ux + 2.0;
% Initialize state
t0 = 0; tf = 30; tstep = 0.001;
t = t0:tstep:tf;
x = zeros(length(t),10);
x_world = zeros(length(t),3); % [N E psi]
x_world(1,2) = 1; % E(t=0)

x(:,3) = speed(end); % Ux
x(1,8) = 0; % s(t=0)
x(1,9) = 1; % e(t=0)
Fx = zeros(1,4);

% Euler integration
i = 1;
while x(i,8) < trackInfo.sLine(6)
    delta = steering(simulation, driver, x(i,:), t(i));
    alpha = slips(simulation, vehicle, x(i,:), delta);
    Fy = tireforces(simulation, vehicle, alpha, Fz);
    dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo);
    dxdt(3) = 0;
    x(i+1,:) = x(i,:) + tstep*dxdt;
    dxdt_world = derivs_world(x(i,:), x_world(i,:));
    x_world(i+1,:) = x_world(i,:) + tstep*dxdt_world;
    i = i + 1;
end

% Add the simulation time for this trial to our t_sim vector
t_sim(length(speed)) = t(i);

% Truncate unused states
x(i+1:end,:) = [];
x_world(i+1:end,:) = [];

% Plot
e = x(:,9);
s = x(:,8);
figure; 
plot(s, e, 'k');
title('Lateral error vs. distance along track');
xlabel('Distance (m)'); ylabel('Lateral error (m)');
legend(['Ux = ', num2str(speed(end)), 'm/s']);

figure; plot(insideLineE, insideLineN, 'k', ...
    outsideLineE, outsideLineN, 'k', x_world(:,2), x_world(:,1), 'k:');
title('Last simulation: world coordinates');
xlabel('East (m)'); ylabel('North (m)');
legend('Track', 'Track', 'Vehicle path', 'Location', 'NorthWest');

figure;
bar(speed, t_sim, 0.1, 'k');
title('Total time to complete track vs. speed');
ylim([min(t_sim) - 1, max(t_sim) + 1])
xlabel('Ux (m/s)'); ylabel('Time (s)');

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw3_prob2.m', options));
%}