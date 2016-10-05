%% Script for 2011 ME227 HW 3 Problem 3
% Author: Ruslan Kurdyumov
% Date: April 20, 2011

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
driver.tmode = 'trail'; % Torque input
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

%% Constant torque
% When we apply a constant torque at the corner exit, we take away friction
% from our lateral force, which keeps us cornering, and apply it to
% longitudinal force, which propels us forward.  This can cause lateral
% slip. Our total time to complete the track goes down until the constant
% torque is too high, which point oscillatory response in Ux and deltapsi
% degrade our time.  Our exit speed follows the same pattern, going up
% until the constant torque is too high.

finTorques = [200 250 300 350];

for k = 1:length(finTorques)
    
    % Initialize state
    t0 = 0; tf = 30; tstep = 0.001;
    t = t0:tstep:tf;
    x = zeros(length(t),10);
    x_world = zeros(length(t),3); % [N E psi]

    Ux_0 = 52;
    x(1,3) = Ux_0; % Ux(t=0)
    x(1,4:7) = Ux_0/vehicle.Re;
    fTorque = finTorques(k);

    % Euler integration
    i = 1;
    while x(i,8) < trackInfo.sLine(6) && abs(x(i,9)) < 50
        delta = steering(simulation, driver, x(i,:), t(i));
        Tq = torques(simulation, driver, x(i,:), trackInfo, fTorque);
        [alpha, kappa] = slips(simulation, vehicle, x(i,:), delta);
        [Fy Fx] = tireforces(simulation, vehicle, alpha, Fz, kappa);
        dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo, Tq);
        x(i+1,:) = x(i,:) + tstep*dxdt;
        dxdt_world = derivs_world(x(i,:), x_world(i,:));
        x_world(i+1,:) = x_world(i,:) + tstep*dxdt_world;
        i = i + 1;
    end
    t_sim(k) = t(i);

    % Truncate unused states
    x(i+1:end,:) = [];
    x_world(i+1:end,:) = [];

    N{k} = x_world(:,1);
    E{k} = x_world(:,2);
    s{k} = x(:,8);
    e{k} = x(:,9);
    Ux{k} = x(:,3);
    Uy{k} = x(end,1);
    delta_psi{k} = x(:,10);
end

marker = {'k', 'k:', 'k--', 'k-.'};
figure;
hold on;
for k = 1:length(finTorques)
    plot(s{k}, e{k}, marker{k});
end
hold off
title('Lateral error vs. distance along track (constant corner exit torque)');
xlabel('s (m)'); ylabel('e (m)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
    [num2str(finTorques(3)), 'N*m'], [num2str(finTorques(4)), 'N*m'], ...
    'Location', 'Best');

figure;
hold on;
for k = 1:length(finTorques)
    plot(s{k}, Ux{k}, marker{k});
end
hold off
title('Ux vs. distance along track (constant corner exit torque)');
xlabel('s (m)'); ylabel('Ux (m/s)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
    [num2str(finTorques(3)), 'N*m'], [num2str(finTorques(4)), 'N*m'], ...
    'Location', 'Best');

figure;
hold on;
for k = 1:length(finTorques)
    plot(s{k}, 180/pi*delta_psi{k}, marker{k});
end
hold off
title('\Delta\psi vs. distance along track (constant corner exit torque)');
xlabel('s (m)'); ylabel('\Delta\psi (deg)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
    [num2str(finTorques(3)), 'N*m'], [num2str(finTorques(4)), 'N*m'], ...
    'Location', 'Best');

figure;
hold on;
for k = 1:length(finTorques)
    plot(s{k}, 180/pi*delta_psi{k}, marker{k});
end
hold off
title('\Delta\psi vs. distance along track (constant corner exit torque) [zoomed]');
xlabel('s (m)'); ylabel('\Delta\psi (deg)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
    [num2str(finTorques(3)), 'N*m'], [num2str(finTorques(4)), 'N*m'], ...
    'Location', 'SouthWest');
ylim([-5 5]);

figure;
bar(finTorques, t_sim(1:length(finTorques)), 0.1, 'k');
title('Total time to complete track vs. final constant corner exit torque');
ylim([min(t_sim) - 1, max(t_sim) + 1])
xlabel('Torque (N*m)'); ylabel('Time (s)');

for i = 1:length(finTorques)
    V(i) = sqrt((Ux{i}(end))^2 + Uy{i}^2);
end
figure;
bar(finTorques, V, 0.1, 'k');
title('Exit speed vs. final constant corner exit torque');
ylim([min(V) - 1, max(V) + 1])
xlabel('Torque (N*m)'); ylabel('Speed (m/s)');

figure; 
hold on;
plot(insideLineE, insideLineN, 'k:', ...
    outsideLineE, outsideLineN, 'k:');
for k = 1:length(finTorques)
    plot(E{k}, N{k}, marker{k});
end
hold off
title('World coordinates: constant corner exit torque');
xlabel('East (m)'); ylabel('North (m)');
legend('Track', 'Track', 'Vehicle paths', 'Location', 'Best');

%% Linear ramp torque
% DeltaPsi > 5 degrees when we use the 1000Nm final torque ramp value.
% Previously, we hit our threshold at 350Nm, so ramping up the torque is
% clearly a better method since we don't want to lose friction in the
% lateral direction immediately, but rather gradually as we exit our
% corner.  Our exit speed goes up and our track time goes down as we
% increase the final ramp torque value.

driver.tmode = 'throttle'  % Torque input
finTorques = [200 600 1000];

for k = 1:length(finTorques)
    
    % Initialize state
    t0 = 0; tf = 30; tstep = 0.001;
    t = t0:tstep:tf;
    x = zeros(length(t),10);
    x_world = zeros(length(t),3); % [N E psi]

    Ux_0 = 52;
    x(1,3) = Ux_0; % Ux(t=0)
    x(1,4:7) = Ux_0/vehicle.Re;
    fTorque = finTorques(k);

    % Euler integration
    i = 1;
    while x(i,8) < trackInfo.sLine(6) && abs(x(i,9)) < 50
        delta = steering(simulation, driver, x(i,:), t(i));
        Tq = torques(simulation, driver, x(i,:), trackInfo, fTorque);
        [alpha, kappa] = slips(simulation, vehicle, x(i,:), delta);
        [Fy Fx] = tireforces(simulation, vehicle, alpha, Fz, kappa);
        dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo, Tq);
        x(i+1,:) = x(i,:) + tstep*dxdt;
        dxdt_world = derivs_world(x(i,:), x_world(i,:));
        x_world(i+1,:) = x_world(i,:) + tstep*dxdt_world;
        i = i + 1;
    end
    t_sim(k) = t(i);

    % Truncate unused states
    x(i+1:end,:) = [];
    x_world(i+1:end,:) = [];

    N{k} = x_world(:,1);
    E{k} = x_world(:,2);
    s{k} = x(:,8);
    e{k} = x(:,9);
    Ux{k} = x(:,3);
    Uy{k} = x(end,1);
    delta_psi{k} = x(:,10);
end
%
figure;
hold on;
for k = 1:length(finTorques)
    plot(s{k}, e{k}, marker{k});
end
hold off
title('Lateral error vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('e (m)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
    [num2str(finTorques(3)), 'N*m'], 'Location', 'Best');

figure;
hold on;
for k = 1:length(finTorques)
    plot(s{k}, Ux{k}, marker{k});
end
hold off
title('Ux vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('Ux (m/s)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
    [num2str(finTorques(3)), 'N*m'], 'Location', 'Best');

figure;
hold on;
for k = 1:length(finTorques)
    plot(s{k}, 180/pi*delta_psi{k}, marker{k});
end
hold off
title('\Delta\psi vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('\Delta\psi (deg)');
legend([num2str(finTorques(1)), 'N*m'], [num2str(finTorques(2)), 'N*m'], ...
    [num2str(finTorques(3)), 'N*m'], 'Location', 'Best');

figure;
bar(finTorques, t_sim(1:length(finTorques)), 0.1, 'k');
title('Total time to complete track vs. final ramp corner exit torque');
ylim([min(t_sim) - 1, max(t_sim) + 1])
xlabel('Torque (N*m)'); ylabel('Time (s)');

for i = 1:length(finTorques)
    V(i) = sqrt((Ux{i}(end))^2 + Uy{i}^2);
end
figure;
bar(finTorques, V(1:length(finTorques)), 0.1, 'k');
title('Exit speed vs. final ramp corner exit torque');
ylim([min(V) - 1, max(V) + 1])
xlabel('Torque (N*m)'); ylabel('Speed (m/s)');

figure; 
hold on;
plot(insideLineE, insideLineN, 'k:', ...
    outsideLineE, outsideLineN, 'k:');
for k = 1:length(finTorques)
    plot(E{k}, N{k}, marker{k});
end
hold off
title('World coordinates: ramp corner exit torque');
xlabel('East (m)'); ylabel('North (m)');
legend('Track', 'Track', 'Vehicle paths', 'Location', 'Best');

%% Initial speed changes: linear ramp torque
% We can increase our initial speed slightly and go through the corner at a
% faster speed.  Increasing the initial speed causes an increase in the
% lateral error and heading error.  Increasing the initial speed also
% causes faster exit speed and track time, up to a limit.

driver.tmode = 'throttle';  % Torque input
Uxi = [51 52 53 54];

for k = 1:length(Uxi)
    
    % Initialize state
    t0 = 0; tf = 30; tstep = 0.001;
    t = t0:tstep:tf;
    x = zeros(length(t),10);
    x_world = zeros(length(t),3); % [N E psi]

    Ux_0 = Uxi(k);
    x(1,3) = Ux_0; % Ux(t=0)
    x(1,4:7) = Ux_0/vehicle.Re;
    fTorque = 600;

    % Euler integration
    i = 1;
    while x(i,8) < trackInfo.sLine(6) && abs(x(i,9)) < 50
        delta = steering(simulation, driver, x(i,:), t(i));
        Tq = torques(simulation, driver, x(i,:), trackInfo, fTorque);
        [alpha, kappa] = slips(simulation, vehicle, x(i,:), delta);
        [Fy Fx] = tireforces(simulation, vehicle, alpha, Fz, kappa);
        dxdt = derivs(simulation, vehicle, driver, x(i,:), Fy, Fx, delta, trackInfo, Tq);
        x(i+1,:) = x(i,:) + tstep*dxdt;
        dxdt_world = derivs_world(x(i,:), x_world(i,:));
        x_world(i+1,:) = x_world(i,:) + tstep*dxdt_world;
        i = i + 1;
    end
    t_sim(k) = t(i);

    % Truncate unused states
    x(i+1:end,:) = [];
    x_world(i+1:end,:) = [];

    N{k} = x_world(:,1);
    E{k} = x_world(:,2);
    s{k} = x(:,8);
    e{k} = x(:,9);
    Ux{k} = x(:,3);
    Uy{k} = x(end,1);
    delta_psi{k} = x(:,10);
end

figure;
hold on;
for k = 1:length(Uxi)
    plot(s{k}, e{k}, marker{k});
end
hold off
title('Lateral error vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('e (m)');
legend([num2str(Uxi(1)), 'm/s'], [num2str(Uxi(2)), 'm/s'], ...
    [num2str(Uxi(3)), 'm/s'], [num2str(Uxi(4)), 'm/s'], ...
    'Location', 'Best');

figure;
hold on;
for k = 1:length(Uxi)
    plot(s{k}, Ux{k}, marker{k});
end
hold off
title('Ux vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('Ux (m/s)');
legend([num2str(Uxi(1)), 'm/s'], [num2str(Uxi(2)), 'm/s'], ...
    [num2str(Uxi(3)), 'm/s'], [num2str(Uxi(4)), 'm/s'], ...
    'Location', 'Best');

figure;
hold on;
for k = 1:length(Uxi)
    plot(s{k}, 180/pi*delta_psi{k}, marker{k});
end
hold off
title('\Delta\psi vs. distance along track (ramp corner exit torque)');
xlabel('s (m)'); ylabel('\Delta\psi (deg)');
legend([num2str(Uxi(1)), 'm/s'], [num2str(Uxi(2)), 'm/s'], ...
    [num2str(Uxi(3)), 'm/s'], [num2str(Uxi(4)), 'm/s'], ...
    'Location', 'Best');

figure;
bar(Uxi, t_sim(1:length(Uxi)), 0.1, 'k');
title('Total time to complete track vs. initial speed');
ylim([min(t_sim) - 1, max(t_sim) + 1])
xlabel('Speed (m/s)'); ylabel('Time (s)');

for i = 1:length(Uxi)
    V(i) = sqrt((Ux{i}(end))^2 + Uy{i}^2);
end
figure;
bar(Uxi, V(1:length(Uxi)), 0.1, 'k');
title('Exit speed vs. initial speed for ramp corner exit torque');
ylim([min(V) - 1, max(V) + 1])
xlabel('Initial speed (m/s)'); ylabel('Final speed (m/s)');

figure; 
hold on;
plot(insideLineE, insideLineN, 'k:', ...
    outsideLineE, outsideLineN, 'k:');
for k = 1:length(Uxi)
    plot(E{k}, N{k}, marker{k});
end
hold off
title('World coordinates: ramp corner exit torque w/ changing initial speed');
xlabel('East (m)'); ylabel('North (m)');
legend('Track', 'Track', 'Vehicle paths', 'Location', 'Best');

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw3_prob3.m', options));
%}