function wheelTorques = torques(simulation,driver,state,trackInfo,fTorque)
% wheelTorques: 
% Author: Ruslan Kurdyumov
% Date: April 4, 2011
%
% Usage: wheelTorques = steering(simtime,state,simulation,driver) where
% simulation is a structure that provides a variable that describes the desired
% drive/brake behavior and supplies all the necessary parameters for the maneuver
% and simtime is the time at which the wheel torque is desired. wheelTorques
% is a vector with the elements enumerated in the standard order.

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% Trail braking 

sLine = trackInfo.sLine;

if length(state) == 10
    s = state(8);
    if s >= sLine(1) && s < sLine(2)
        wheelTorques(lf) = -1000;
        wheelTorques(rf) = -1000;
        wheelTorques(lr) = -500;
        wheelTorques(rr) = -500;
    elseif s >= sLine(2) && s < sLine(3)
        wheelTorques(lf) = -1000 + (10-(-1000))/(301.87 - 20)*(s-20);
        wheelTorques(rf) = -1000 + (10-(-1000))/(301.87 - 20)*(s-20);
        wheelTorques(lr) = -500 + (10-(-500))/(301.87 - 20)*(s-20);
        wheelTorques(rr) = -500 + (10-(-500))/(301.87 - 20)*(s-20);
    elseif s >= sLine(3) && s < sLine(4)
        wheelTorques(lf) = 10;
        wheelTorques(rf) = 10;
        wheelTorques(lr) = 10;
        wheelTorques(rr) = 10;
    elseif s >= sLine(4) && s < sLine(5)
        switch lower(driver.tmode)
            case 'trail'
                wheelTorques(lf) = fTorque;
                wheelTorques(rf) = fTorque;
                wheelTorques(lr) = fTorque;
                wheelTorques(rr) = fTorque;
            case 'throttle'
                wheelTorques(lf) = 10 + (fTorque - 10)/(600.32-318.45)*(s - 318.45);
                wheelTorques(rf) = 10 + (fTorque - 10)/(600.32-318.45)*(s - 318.45);
                wheelTorques(lr) = 10 + (fTorque - 10)/(600.32-318.45)*(s - 318.45);
                wheelTorques(rr) = 10 + (fTorque - 10)/(600.32-318.45)*(s - 318.45);
            otherwise
                error('Not a valid drive mode');
        end
    elseif s >= sLine(5) && s < sLine(6)
        wheelTorques(lf) = 200;
        wheelTorques(rf) = 200;
        wheelTorques(lr) = 200;
        wheelTorques(rr) = 200;
    end 

end
