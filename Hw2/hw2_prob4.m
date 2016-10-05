%% Script for 2011 ME227 HW 2 Problem 4
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
driver.mode = 'step';	% Type of steering (control strategy or maneuver)
driver.delta0 = 0;      % Initial steering angle
driver.deltaf =	1*pi/180; % Final steering angle (rad)
driver.steertime = 0;	% Time to start the step
simulation.tmodel = 'linear';
simulation.vmodel = 'bike'; 

% Define any convenient physical parameters
simulation.g = 9.81;


%% (2.4-1) Root locus and Yaw rate step response: standard TTS

a = vehicle.a; b = vehicle.b; Izz = vehicle.Izz; m = vehicle.m;
L = vehicle.L; g = simulation.g;
C_af = vehicle.Ca(lf) +  vehicle.Ca(rf);
C_ar = vehicle.Ca(lr) +  vehicle.Ca(rr);
C_0 = C_ar + C_af;
C_1 = a*C_af - b*C_ar;
C_2 = a^2*C_af + b^2*C_ar;
n = 1000;
V = linspace(1,40,n);
p = zeros(n,2);

for i = 1:size(V,2)
    num = [a*C_af, (a*C_af*C_0 - C_1*C_af)/(m*V(i))];
    den = [Izz, (C_0*Izz + m*C_2)/(m*V(i)), ...
        C_0*C_2/(m*V(i)^2) - C_1 - C_1^2/(m*V(i)^2)];
    H = tf(num, den);
    p(i,:) = pole(H);
end

p_r = real(p);
p_i = imag(p);
figure;
plot(p_r(:,1), p_i(:,1), 'k-', p_r(:,2), p_i(:,2), 'k-'); 
title('Root locus w/ increasing speed 1m/s - 40m/s (standard TTS)');
xlabel('Real'); ylabel('Imaginary')

% Step response
d = 1*pi/180;   % Step amplitude

plot_opt = {'k', 'k--', 'k:', 'k-.'};
V = [10 20 30 40];
figure; hold on;
for i = 1:size(V,2)
    num = [a*C_af, (a*C_af*C_0 - C_1*C_af)/(m*V(i))];
    den = [Izz, (C_0*Izz + m*C_2)/(m*V(i)), ...
        C_0*C_2/(m*V(i)^2) - C_1 - C_1^2/(m*V(i)^2)];
    H = tf(num, den);
    step(d*H, plot_opt{i})
end
hold off;

title('Yaw rate response to 1 deg. step steer (standard TTS)');
ylabel('yaw rate (rad/s)');
grid off;
legend('10m/s', '20m/s', '30m/s', '40m/s', 'Location', 'SouthEast')

%{
[y t] = step(d*H_10,1);
% Compare the predicted and actual steady state values
K = b*m/(L*C_af) - a*m/(L*C_ar);
r_ss = V/(L + K*V^2)*d
r_last = y(end)
%}

%% (2.4-2) Root locus and Yaw rate step response: 65/35 weight

vehicle.a = 0.35*vehicle.L; vehicle.b = 0.65*vehicle.L;
a = vehicle.a; b = vehicle.b;
C_0 = C_ar + C_af;
C_1 = a*C_af - b*C_ar;
C_2 = a^2*C_af + b^2*C_ar;
n = 1000;
V = linspace(1,40,n);
p = zeros(n,2);

for i = 1:size(V,2)
    num = [a*C_af, (a*C_af*C_0 - C_1*C_af)/(m*V(i))];
    den = [Izz, (C_0*Izz + m*C_2)/(m*V(i)), ...
        C_0*C_2/(m*V(i)^2) - C_1 - C_1^2/(m*V(i)^2)];
    H = tf(num, den);
    p(i,:) = pole(H);
end

p_r = real(p);
p_i = imag(p);
figure;
plot(p_r(:,1), p_i(:,1), 'k-', p_r(:,2), p_i(:,2), 'k-'); 
title('Root locus w/ increasing speed 1m/s - 40m/s (65/35 weight)');
xlabel('Real'); ylabel('Imaginary')

% Step response
d = 1*pi/180;   % Step amplitude

plot_opt = {'k', 'k--', 'k:', 'k-.'};
V = [10 20 30 40];
figure; hold on;
for i = 1:size(V,2)
    num = [a*C_af, (a*C_af*C_0 - C_1*C_af)/(m*V(i))];
    den = [Izz, (C_0*Izz + m*C_2)/(m*V(i)), ...
        C_0*C_2/(m*V(i)^2) - C_1 - C_1^2/(m*V(i)^2)];
    H = tf(num, den);
    step(d*H, plot_opt{i})
end
hold off;

title('Yaw rate response to 1 deg. step steer (65/35 weight)');
ylabel('yaw rate (rad/s)');
grid off;
legend('10m/s', '20m/s', '30m/s', '40m/s', 'Location', 'SouthEast')

%% (2.4-3) Root locus and Yaw rate step response: 52/48 weight
% As the vehicle became more front heavy, the root locus moved out from the 
% real axis, so the step response was not as heavily damped.  As the 
% vehicle became less front heavy, the root locus shifted completely to the
% negative real axis, so the step response was overdamped even as we 
% increased the vehicle speed.

vehicle.a = 0.48*vehicle.L; vehicle.b = 0.52*vehicle.L;
a = vehicle.a; b = vehicle.b;
C_0 = C_ar + C_af;
C_1 = a*C_af - b*C_ar;
C_2 = a^2*C_af + b^2*C_ar;
n = 1000;
V = linspace(1,40,n);
p = zeros(n,2);

for i = 1:size(V,2)
    num = [a*C_af, (a*C_af*C_0 - C_1*C_af)/(m*V(i))];
    den = [Izz, (C_0*Izz + m*C_2)/(m*V(i)), ...
        C_0*C_2/(m*V(i)^2) - C_1 - C_1^2/(m*V(i)^2)];
    H = tf(num, den);
    p(i,:) = pole(H);
end

p_r = real(p);
p_i = imag(p);
figure;
plot(p_r(:,1), p_i(:,1), 'k-', p_r(:,2), p_i(:,2), 'k-'); 
title('Root locus w/ increasing speed 1m/s - 40m/s (52/48 weight)');
xlabel('Real'); ylabel('Imaginary')

% Step response
d = 1*pi/180;   % Step amplitude

plot_opt = {'k', 'k--', 'k:', 'k-.'};
V = [10 20 30 40];
figure; hold on;
for i = 1:size(V,2)
    num = [a*C_af, (a*C_af*C_0 - C_1*C_af)/(m*V(i))];
    den = [Izz, (C_0*Izz + m*C_2)/(m*V(i)), ...
        C_0*C_2/(m*V(i)^2) - C_1 - C_1^2/(m*V(i)^2)];
    H = tf(num, den);
    step(d*H, plot_opt{i})
end
hold off;

title('Yaw rate response to 1 deg. step steer (52/48 weight)');
ylabel('yaw rate (rad/s)');
grid off;
legend('10m/s', '20m/s', '30m/s', '40m/s', 'Location', 'SouthEast')

% Copy-paste into command prompt to publish
%{ 
options.format = 'pdf';
options.showCode = false;
open(publish('hw2_prob4.m', options));
%}