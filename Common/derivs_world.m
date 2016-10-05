function dxdt_world = derivs_world(state, state_world)
% derivs_world: calculates absolute position state derivatives (N, E, psi)
% Author: Ruslan Kurdyumov
% Date: April 20, 2011
% 
% Usage: dxdt = derivs(simulation,vehicle,state,driver, varargin)
% where state and state_world are defined by the states being used in the 
% simulation. The output is a vector of the absolute position state 
% derivatives of the system [N E psi].

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

if length(state) == 10 && length(state_world) == 3
    Uy = state(1);
    r = state(2);
    Ux = state(3);
    psi = state_world(3);
    % Calculate derivatives
    Ndot = Ux*cos(psi) - Uy*sin(psi);
    Edot = -Uy*cos(psi) - Ux*sin(psi);
    psidot = r;
    % Output
    dxdt_world(1) = Ndot;
    dxdt_world(2) = Edot;
    dxdt_world(3) = psidot;
end