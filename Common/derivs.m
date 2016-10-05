function dxdt = derivs(simulation,vehicle,driver,state,varargin)
% derivs: calculates state derivatives (Ux and r)
% Author: Ruslan Kurdyumov
% Date: April 4, 2011
% 
% Usage: dxdt = derivs(simulation,vehicle,state,driver, varargin)
% where simulation is a structure that supplies necessary information about the
% simulation environment, vehicle is a structure that describes the physical
% properties of the vehicle being simulated, state is defined by the
% states being used in the simulation and varargin contains the necessary vector
% of the forces on the vehicle. For the 'bike' model, varargin = Fy.  For
% the fourwheel model, varargin{1} = Fy, varargin{2} = Fx, and varargin{3} = 
% delta. The output is a vector of the state derivatives of the system 
% described by a bicycle vehicle model.

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% Here we use a "switch" statement to determine which derivatives we want
% the code to generate. The function 'lower' is used to help make the function
% more robust as it will convert the input string to lower-case
switch lower(simulation.vmodel)
    % Bicycle Model
    case 'bike'             
        if all(isfield(vehicle, {'a', 'b', 'm', 'Izz'})) && ...
            length(varargin{1}) == 4 && isfield(simulation, 'speed')
            % Get the variables we will need
            if length(state) == 2
                Ux = simulation.speed;
            else
                Ux = state(3);
            end
            r = state(2);
            a = vehicle.a; b = vehicle.b; m = vehicle.m; Izz = vehicle.Izz;
            Fy = varargin{1};
            Fyf = Fy(lf) + Fy(rf);
            Fyr = Fy(lr) + Fy(rr);
            rdot = (a*Fyf - b*Fyr) / Izz;
            Uydot = (Fyf + Fyr)/m - r*Ux;
            dxdt(1) = Uydot;
            dxdt(2) = rdot;
            % Additional states when following path
            switch lower(driver.mode)
                case 'path'
                    trackInfo = varargin{4};
                    Uy = state(1); delta_psi = state(10); s = state(8);
                    % Calculate the current curvature
                    rhoInv = track(trackInfo,s);
                    sdot = Ux*cos(delta_psi) - Uy*sin(delta_psi);
                    edot = Uy*cos(delta_psi) + Ux*sin(delta_psi);
                    delta_psidot = r - sdot*rhoInv;
                    dxdt(8) = sdot;
                    dxdt(9) = edot;
                    dxdt(10) = delta_psidot;
                    % Don't update the rest of the states yet
                    dxdt(3:7) = 0;
                otherwise
                    % Don't need to add states
            end
        else                            
            error('Missing bike model deriv parameters');
        end

    case 'fourwheel'
        if all(isfield(vehicle, {'a', 'b', 'm', 'Izz', 'd'}))
            % Get the variables we will need
            if length(state) == 2
                Ux = simulation.speed;
            else
                Ux = state(3);
            end
            Uy = state(1); r = state(2);
            a = vehicle.a; b = vehicle.b; m = vehicle.m; 
            Izz = vehicle.Izz; d = vehicle.d;
            Fy = varargin{1}; Fx = varargin{2}; delta = varargin{3};
            % y-direction
            Fyr = Fy(lr) + Fy(rr);
            Fyf = Fy(lf)*cos(delta(lf)) + Fx(lf)*sin(delta(lf)) + ...
                Fy(rf)*cos(delta(rf)) + Fx(rf)*sin(delta(rf));
            Uydot = (Fyf + Fyr)/m - r*Ux;
            % x-direction
            Fxr = Fx(lr) + Fx(rr);
            Fxf = Fx(lf)*cos(delta(lf)) + Fx(rf)*cos(delta(rf)) - ...
                Fy(lf)*sin(delta(lf)) - Fy(rf)*sin(delta(rf));
            Uxdot = (Fxf + Fxr)/m + r*Uy;
            % Calculate moments
            Fa = Fy(lf)*cos(delta(lf)) + Fy(rf)*cos(delta(rf)) + ...
                Fx(lf)*sin(delta(lf)) + Fx(rf)*sin(delta(rf));
            Fb = Fy(lr) + Fy(rr);
            Fd = Fx(rr) - Fx(lr) + Fx(rf)*cos(delta(rf)) - ...
                Fx(lf)*cos(delta(lf)) + Fy(lf)*sin(delta(lf)) - ...
                Fy(rf)*sin(delta(rf));
            rdot = (a*Fa - b*Fb + d/2*Fd)/Izz;
            dxdt(1) = Uydot;
            dxdt(2) = rdot;
            if length(state) > 2
                dxdt(3) = Uxdot;
            end
            % Additional states when following path
            switch lower(driver.mode)
                case 'path'
                    trackInfo = varargin{4};
                    Uy = state(1); delta_psi = state(10); s = state(8);
                    Re = vehicle.Re; Jw = vehicle.Jw;
                    % Calculate the current curvature
                    rhoInv = track(trackInfo,s);
                    sdot = Ux*cos(delta_psi) - Uy*sin(delta_psi);
                    edot = Uy*cos(delta_psi) + Ux*sin(delta_psi);
                    delta_psidot = r - sdot*rhoInv;
                    dxdt(8) = sdot;
                    dxdt(9) = edot;
                    dxdt(10) = delta_psidot;
                    if length(varargin) > 4
                        Tq = varargin{5};
                        for w = lf:rr
                            wdot(w) = (Tq(w) - Re*Fx(w))/Jw;
                        end
                        dxdt(4:7) = wdot;
                    else
                        dxdt(4:7) = 0;
                    end
                otherwise
                    % Don't need to add states
            end
        else
            error('Missing fourwheel model deriv parameters');
        end
    otherwise
        error('Not a valid vehicle model')
end
