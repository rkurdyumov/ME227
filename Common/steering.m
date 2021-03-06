function delta = steering(simulation,driver,state,simtime)
% steering: calculates desired input steering behavior
% Author: Ruslan Kurdyumov
% Date: April 4, 2011
%
% Usage: delta = steering(simulation,driver,state,simtime) where simulation is a
% structure that provides a variable that describes the desired steering
% behavior and supplies all the necessary parameters for the maneuver and
% simtime is the time at which the steering input is desired. delta is a vector
% with the elements enumerated in the standard order.

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% Here we use a "switch" statement to determine which type of maneuver we want
% the code to generate. The function 'lower' is used to help make the function
% more robust as it will convert the input string to lower-case
switch lower(driver.mode)
    % Step input
    case 'step'             
        if all(isfield(driver, {'delta0', 'deltaf', 'steertime'})) 
            if simtime < driver.steertime
                delta(lf) = driver.delta0;
                delta(rf) = driver.delta0;
                delta(lr) = 0;
                delta(rr) = 0;
            else
                delta(lf) = driver.deltaf;
                delta(rf) = driver.deltaf;
                delta(lr) = 0;
                delta(rr) = 0;
            end
        else                            
            error('Missing step model steering parameters');
        end
    
    case 'data'    
        if all(isfield(driver, {'t_exp', 'delta_exp'}))
            delta(lf) = interp1q(driver.t_exp, driver.delta_exp, simtime);
            delta(rf) = interp1q(driver.t_exp, driver.delta_exp, simtime);
            delta(lr) = 0;
            delta(rr) = 0;
        else                            
            error('Missing data model steering parameters');
        end
        
    % Ramp input
    case 'ramp'             
        if all(isfield(driver, {'delta0', 'steertime', 'deltaramp'})) 
            if simtime < driver.steertime
                delta(lf) = driver.delta0;
                delta(rf) = driver.delta0;
                delta(lr) = 0;
                delta(rr) = 0;
            else
                delta(lf) = (simtime - driver.steertime)*driver.deltaramp ...
                    + driver.delta0;
                delta(rf) = (simtime - driver.steertime)*driver.deltaramp ...
                    + driver.delta0;
                delta(lr) = 0;
                delta(rr) = 0;
            end
        else                            
            error('Missing step model steering parameters');
        end
        
    % Path tracking
    case 'path'
        if all(isfield(driver, {'Ke', 'Kpsi'}))
            e = state(9); delta_psi = state(10);
            Ke = driver.Ke; Kpsi = driver.Kpsi;
            delta(lf) = -Ke*e - Kpsi*delta_psi;
            delta(rf) = -Ke*e - Kpsi*delta_psi;
            delta(lr) = 0;
            delta(rr) = 0;
        else
            error('Missing path model steering parameters');
        end
        
    % Drifting
    case 'drifting'
        if all(isfield(driver, {'deltaEq', 'UyEq', 'rEq', 'KUy', 'Kr'}))
            deltaEq = driver.deltaEq; UyEq = driver.UyEq; rEq = driver.rEq;
            KUy = driver.KUy; Kr = driver.Kr;
            Uy = state(1); r = state(2);
            deltadelta = -KUy*(Uy - UyEq) - Kr*(r - rEq);
            delta(lf) = deltaEq + deltadelta;
            delta(rf) = deltaEq + deltadelta;
            delta(lr) = 0;
            delta(rr) = 0;
        else
            error('Missing path model steering parameters');
        end

    otherwise
        error('Not a valid drive mode')
end
