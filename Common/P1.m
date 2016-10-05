% Vehicle Model for P1
% Author: Ruslan Kurdyumov
% Date: April 27, 2011

P1.m = 1724; %mass (kg)
P1.Ca = [37500 37500 67500 67500]'; %Cornering stiffness (N/rad)
P1.Izz = 1300; %Yaw inertia (kg-m^2)
P1.a = 1.35; %Distance from CG to front axle (m)
P1.b = 1.15; %Distance from CG to rear axle (m)
P1.L = 2.5; %Wheelbase (m)
P1.mu_peak = [0.55 0.55 0.53 0.53]'; %Peak friction coeff
P1.mu_slide = [0.55 0.55 0.53 0.53]'; %Sliding friction coeff