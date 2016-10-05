% Vehicle Model for Audi TTs
% Author: Ruslan Kurdyumov
% Date: April 4, 2011

% Define a MATLAB structure in this file that specifies the physical values for
% a vehicle.  For convenience, we ask that you call this stucture 'vehicle'.
% For example, specify the vehicle's mass this way:
vehicle.m = 1466.9;             % mass (kg) 
vehicle.Ca = 1/2*[200000; 200000;...
    160000; 160000];            % wheel cornering stiffnesses (N/rad)
vehicle.Izz = 2235;             % moment of inertia (kg m^2)
vehicle.L = 2.468;              % wheelbase (m)
vehicle.a = 0.4125*vehicle.L;	% length from front axle to CG
vehicle.b = 0.5875*vehicle.L;	% length from rear axle to CG
vehicle.d = 1.554;              % track length (m)
vehicle.mu_peak = [1.2; 1.2; 1.2; 1.2];     % Peak tire-road friction coefficient
vehicle.mu_slide = [0.8; 0.8; 0.8; 0.8];    % Sliding tire-road friction coefficient
vehicle.Jw = 1.2;               % wheel inertia (kg*m^2)
vehicle.Re = 0.3434;            % effective radius (m)
vehicle.Cx = [200000; 200000;...
    200000; 200000];            % longitudinal cornering stiffness (N/rad)