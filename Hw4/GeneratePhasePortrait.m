function stateDot = GeneratePhasePortrait(uy,r,mode,desiredStateDerivative)
% GeneratePhasePortrait interfaces to PPLANE7 phase portrait
% plotting by providing computations of a vehicle's state derivatives in a batch format
%as required by PPLANE7
% Author: Rami Hindiyeh
% Date: April 19, 2011

%Define tire position indices
lf = 1; rf = 2; lr = 3; rr = 4;

%Vehicle parameters
P1.m = 1724; %mass in kg
P1.Ca = [75000/2; 75000/2; 135000/2; 135000/2]; %cornering stiffnesses in N/rad
P1.Izz = 1300; %Yaw inertia in kg-m^2
P1.a = 1.35; %Distance from CG to front axle in m
P1.b = 1.15; %Distance from CG to rear axle in m
P1.L = P1.a + P1.b; %Wheelbase in m
P1.mu_peak = [0.55 0.55 0.53 0.53]'; %Peak friction coefficient
P1.mu_slide = [0.55 0.55 0.53 0.53]'; %sliding friction coefficient

%Define constant delta (must be fixed for phase portrait)
delta = -10*pi/180*[1 1 0 0]';

%Set simulation parameters
simulation.g = 9.81;
simulation.speed = 8;

%Static normal loads
Fz = [0.5*P1.b*P1.m*simulation.g/P1.L 0.5*P1.b*P1.m*simulation.g/P1.L ...
    0.5*P1.a*P1.m*simulation.g/P1.L 0.5*P1.a*P1.m*simulation.g/P1.L]';

% Wf = Fz(lf) + Fz(rf);
% Wr = Fz(lr) + Fz(rr);
% Cf = P1.Ca(lf) + P1.Ca(rf);
% Cr = P1.Ca(lr) + P1.Ca(rr);
% K = Wf/Cf - Wr/Cr

%'linear' simulation mode = bike model with linear tires. 'nonlinear'
%simulation mode = four wheel model with fiala lateral tire model
switch(lower(mode))
    case 'linear'
        simulation.vmodel = 'bike';
        simulation.tmodel = 'linear';
    case 'nonlinear'
%         simulation.vmodel = 'fourwheel';
        simulation.vmodel = 'bike';
        simulation.tmodel = 'fialalat';
    otherwise
        error('INVALID PHASE PORTRAIT MODE SPECIFIED')
end

%Store Uy and r inputs into state vector;
state = [uy; r];

%Compute slip angles
alpha = slipsPP(simulation,P1,state,delta);
%Compute tire lateral forces and state derivatives
switch(lower(mode))
    case 'linear'
        Fy = tireforcesPP(simulation,P1,alpha);
        dxdt = derivsPP(simulation,P1,state,Fy);
    case 'nonlinear'
        Fy = tireforcesPP(simulation,P1,alpha,Fz);
%         dxdt = derivsPP(simulation,P1,state,Fy,delta);
        dxdt = derivsPP(simulation,P1,state,Fy);

    otherwise
        error('INVALID PHASE PORTRAIT MODE SPECIFIED')
end

%Based on desiredState input, output desired state derivatives
switch(lower(desiredStateDerivative))
    case 'uydot'
        stateDot = dxdt(1,:);
    case 'rdot'
        stateDot = dxdt(2,:);
    otherwise
        error('INVALID DESIRED STATE DERIVATIVE SPECIFIED')
end


