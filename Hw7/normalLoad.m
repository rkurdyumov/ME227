function WeightLoad = normalLoad(vehicle,simulation,ay,Kphi,Fz)
% WeightLoad = RollAngles(vehicle,simulation,ay,Kphi,Fz) returns the normal 
% load at each wheel.
%
% Weightload is the load (N) at a given wheel.  Vehicle contains the 
% vehicle parameters needed, simulation contains simulation.g, ay is the 
% lateral acceleration (m/s^2), Kphi is the roll stiffness at each axle
% (N*m/rad), and Fz is the normal loading (N) of the wheels when static.

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% Define Vehicle Parameters
m = vehicle.m;
ms = vehicle.msprung;
hprime = vehicle.hprime;
hr = vehicle.hr;
hf = vehicle.hf;
a = vehicle.a;
b = vehicle.b;
L = vehicle.L;
Tr = vehicle.d;
Tf = vehicle.d;
g = simulation.g; 

%  Determine Front/Rear/Total Roll Stiffness
KphiTotal = Kphi(1) + Kphi(2);

% ay is in m/s^2
% Calculate Roll Angle
for wheel = (lf:rr)
    Phi(wheel) = (ms*ay*hprime)/(KphiTotal - ms*g*hprime);
end

% Calculate The New Loads on Each Tire due to Weight Transfer
WeightLoad(lf) = Fz(lf) - (1/Tf)*(Kphi(1)*Phi(lf) + m*(b/L)*hf*ay);
WeightLoad(rf) = Fz(rf) + (1/Tf)*(Kphi(1)*Phi(rf) + m*(b/L)*hf*ay);
WeightLoad(lr) = Fz(lr) - (1/Tr)*(Kphi(2)*Phi(lr) + m*(a/L)*hr*ay);
WeightLoad(rr) = Fz(rr) + (1/Tr)*(Kphi(2)*Phi(rr) + m*(a/L)*hr*ay);