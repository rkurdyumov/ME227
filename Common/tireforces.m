function varargout = tireforces(simulation,vehicle,varargin)
% tireforces: calculates force on each tire based on tire model
% Author: Ruslan Kurdyumov
% Date: April 4, 2011
%
% Usage: Changes depending on the input parameters.  If simulation.tmodel 
% indicates that the desired tire model is linear, then the syntax for calling
% this function should be: latForceVariable =
% tireforces(simulation,vehicle,varargin) which returns a linear tire force
% Fy = -Ca*alpha.  However, if simulation.tmodel indicates a nonlinear model,
% possibly including coupled lateral and longitudinal forces, then the usage
% could change to something like: [latForceVariable,longForceVariable] =
% tireforces(simulation,vehicle,varargin) which returns both lateral and
% longitudinal nonlinear tire forces according to the Fiala tire model.

% Enumerate the wheels (this should appear in all of your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% Here we use a "switch" statement to determine which type of tire model gets
% used to calculate the tire forces. The function 'lower' is used to help make
% the function more robust as it will convert the input string to lower-case
switch lower(simulation.tmodel)
    % Linear tire model
    case 'linear'
        if isfield(vehicle, 'Ca') && length(varargin{1}) == 4
            Ca = vehicle.Ca;
            alpha = varargin{1};
            Fy = zeros(1,4);
            for wheel = lf:rr
                Fy(wheel) = -Ca(wheel) * alpha(wheel);
            end
            varargout{1} = Fy;
        else
            error('Missing linear model tireforces parameters');
        end
    
    % Fiala nonlinear tire model
    case 'fiala'
        if all(isfield(vehicle, {'Ca', 'mu_peak', 'mu_slide'})) && ...
            length(varargin{1}) == 4 && length(varargin{2}) == 4
        
            Ca = vehicle.Ca;
            mu = vehicle.mu_peak;
            mu_s = vehicle.mu_slide;
            alpha = varargin{1};
            Fz = varargin{2};
            Fy = zeros(1,4);
            % Only Fy
            if length(varargin) == 2
                for w = lf:rr
                    if abs(tan(alpha(w))) < 3*mu(w)*Fz(w)/Ca(w)
                        Fy(w) = -Ca(w) * tan(alpha(w)) + ...
                        (Ca(w))^2 / (3*mu(w)*Fz(w)) * (2 - mu_s(w)/mu(w)) * ...
                        abs(tan(alpha(w)))*tan(alpha(w)) - (Ca(w))^3 / ...
                        (9*mu(w)^2*Fz(w)^2) * tan(alpha(w))^3 * ...
                        (1 - 2*mu_s(w) / (3*mu(w)));
                    else
                        Fy(w) = -mu_s(w)*Fz(w)*sign(alpha(w));
                    end
                end
            % Combined Fx and Fy model
            elseif length(varargin) == 3
                Cx = vehicle.Cx;
                K = varargin{3};
                Fx = zeros(1,4); f = zeros(1,4); F = zeros(1,4);
                for w = lf:rr
                    f(w) = sqrt((Cx(w)^2)*(K(w)/(1 + K(w)))^2 + ...
                        (Ca(w)^2)*(tan(alpha(w))/(1 + K(w)))^2);
                    if f(w) <= 3*mu(w)*Fz(w)
                        F(w) = f(w) - 1/(3*mu(w)*Fz(w)) * ...
                            (2 - mu_s(w)/mu(w))*f(w)^2 +  ...
                            1/(9*mu(w)^2*Fz(w)^2) * ...
                            (1 - 2*mu_s(w) / (3*mu(w)))*f(w)^3;
                    else
                        F(w) = mu_s(w)*Fz(w);
                    end
                    Fx(w) = Cx(w)*(K(w)/(1 + K(w)))/f(w)*F(w);
                    if isnan(Fx(w))
                        Fx(w) = 0;
                    end
                    Fy(w) = -Ca(w)*(tan(alpha(w))/(1 + K(w)))/f(w)*F(w);
                    if isnan(Fy(w))
                        Fy(w) = 0;
                    end
                end
                varargout{2} = Fx;
            end
            varargout{1} = Fy;    
        else
            error('Missing Fiala model tireforces parameters');
        end

    otherwise 
        error('Not a valid tire model');       % Another error check
end
