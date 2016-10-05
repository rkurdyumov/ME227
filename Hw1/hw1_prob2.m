%% Script for 2011 ME227 HW 1 Problem 2
% Author: Ruslan Kurdyumov
% Date: April 6, 2011

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
driver.mode = 'step';	% Type of steering (control strategy or maneuver)
driver.delta0 = 0;      % Initial steering angle
driver.deltaf =	1*pi/180; % Final steering angle (rad)
driver.steertime = 0;	% Time to start the step
simulation.speed = 20;	% Perform the maneuver at 20 m/s

% Define which models we want to use
simulation.vmodel = 'bike';  % vehicle model

% Define any convenient physical parameters
simulation.g = 9.81;

% Calculate the normal forces for each wheel
Fz(lf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rf) = vehicle.b/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(lr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;
Fz(rr) = vehicle.a/vehicle.L*(vehicle.m * simulation.g)/2;

% Simulate the linear and fiala tire models
a_step = 0.05; a_final = 35;
angle = 0:a_step:a_final;
for i=1:size(angle,2)
    simulation.tmodel = 'linear'; 
    FyL(i,:) = tireforces(simulation,vehicle,angle(i)*pi/180.*ones(1,4));
    simulation.tmodel = 'fiala';
    FyNL(i,:) = tireforces(simulation,vehicle,angle(i)*pi/180.*ones(1,4),Fz);    
end

%% (2.1) Linear & Nonlinear tire curves (Lateral force vs. slip angle) 
figure; plot(angle, FyL(:,1)+FyL(:,2), 'k:', angle, FyNL(:,1) + FyNL(:,2), 'k.-');
title('Total lateral force (front)');
xlabel('time (s)'); ylabel('yaw rate (rad/s)');
legend('Linear', 'Nonlinear', 'Location', 'SouthWest');

figure; plot(angle, FyL(:,3)+FyL(:,4), 'k:', angle, FyNL(:,3) + FyNL(:,4), 'k.-');
title('Total lateral force (rear)');
xlabel('time (s)'); ylabel('yaw rate (rad/s)');
legend('Linear', 'Nonlinear', 'Location', 'SouthWest');

%% (2.2) Check tire curves

disp('a. The curves look smooth and don''t have sudden jumps');
disp(' ');

% b. Calculate slopes (Newtons/degrees)
disp('b. The initial slopes are fairly close: ')
final_angle = 1; % degreess
disp('Front linear slope (N/deg):')
disp(mean(diff(FyL(:,1))) / a_step * pi / 180)
disp('Front nonlinear slope (N/deg), 0-1 deg:')
disp(mean(diff(FyNL(1:final_angle/a_step,1))) / a_step * pi / 180)
disp('Rear linear slope (N/deg):')
disp(mean(diff(FyL(:,3))) / a_step * pi / 180)
disp('Rear nonlinear slope (N/deg), 0-1 deg:')
disp(mean(diff(FyNL(1:final_angle/a_step,3))) / a_step * pi / 180)
disp(' ');

% c. Calculate steady state lateral forces
disp('c. The final force values match exactly:')
disp('Expected final Fyf = -mu_s*b/2L*mg: ')
disp(-vehicle.mu_slide(1)*vehicle.b/vehicle.L*0.5*vehicle.m*simulation.g)
disp('Actual final Fyf:')
disp(FyNL(end,1))
disp('Expected final Fyr =  -mu_s*a/2L*mg: ')
disp(-vehicle.mu_slide(3)*vehicle.a/vehicle.L*0.5*vehicle.m*simulation.g)
disp('Actual final Fyr:')
disp(FyNL(end,3))

%% (2.3) Linear vs. nonlinear yaw rate response 

% Initialize our time vector and state vector
t0 = 0; tf = 1; tstep = 0.01;
t = t0:tstep:tf;
xL = zeros((tf - t0) / tstep + 1,2);
xNL = zeros((tf - t0) / tstep + 1,2);

% Loop until we have error > 10% of linear yaw rate
while abs(0.1*(xL(end,2))) >= abs(xL(end,2) - xNL(end,2))
    % Euler integration
    for i = 1:(size(t,2) - 1)
        simulation.tmodel = 'linear'; 
        deltaL = steering(simulation, driver, xL(i,:), t(i));
        alphaL = slips(simulation, vehicle, xL(i,:), deltaL);
        FyL = tireforces(simulation, vehicle, alphaL);
        dxdtL = derivs(simulation, vehicle, xL(i,:), FyL);
        xL(i+1,:) = xL(i,:) + tstep*dxdtL;

        simulation.tmodel = 'fiala';
        deltaNL = steering(simulation, driver, xNL(i,:), t(i));
        alphaNL = slips(simulation, vehicle, xNL(i,:), deltaNL);
        FyNL = tireforces(simulation, vehicle, alphaNL, Fz);
        dxdtNL = derivs(simulation, vehicle, xNL(i,:), FyNL);
        xNL(i+1,:) = xNL(i,:) + tstep*dxdtNL;
    end
    % Increment the step size of steering angle
    driver.deltaf = driver.deltaf + 0.1*pi/180;
end

% Get the steering angle that caused excessive error, display lateral force
step_angle = driver.deltaf * 180/pi;
disp(['Lateral acceleration (linear) at ', num2str(step_angle),  ' degrees:']);
disp(dxdtL(1) + xL(end,2)*simulation.speed);
disp('derived from a_y = U_y'' + r*U_x')

% Plot the yaw rate response
figure; plot(t, xL(:,2), 'k:', t, xNL(:,2), 'k.');
title(['Yaw rate response at ', num2str(step_angle), ' deg']);
xlabel('time (s)'); ylabel('yaw rate (rad/s)');
legend('Linear', 'Nonlinear');

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw1_prob2.m', options));
%}