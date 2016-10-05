function rhoInv = track(trackInfo, s)
% track: calculates track curvature based on rate of curvature change and
% travelled distance
% Author: Ruslan Kurdyumov
% Date: April 17, 2011
%
% Usage: rhoInv = track(c,s) where s is the travelled distance measured 
% from the beginning of the clothoid.  The track curvature is returned as 
% a single value based on the given inputs.  The trackInfo contains 
% the constant radius (radius), rate of curvature change (c), and the 6
% travelled distance transitions (sLine).

c = trackInfo.c;
rho = trackInfo.radius;
sLine = trackInfo.sLine;

% Straight
if s >= sLine(1) && s <= sLine(2)
    rhoInv = 0;
% Corner entry    
elseif s > sLine(2) && s <= sLine(3)
    rhoInv = 2*c^2*(s - sLine(2));
% Constant radius
elseif s > sLine(3) && s <= sLine(4)
    rhoInv = 1/rho;
% Clothoid exit
elseif s > sLine(4) && s <= sLine(5)
    rhoInv = 2*c^2*(sLine(5) - s);
% Straight
elseif s > sLine(5) && s<= sLine(6)
    rhoInv = 0;
else
    error('Requested curvature for invalid travelled distance')
end