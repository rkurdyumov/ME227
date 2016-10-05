function [datavector]  = wishbone(ruarmyc,ruarmzc,rlarmyc,rlarmzc,r2,r3,r4,r5,zcarcenter,carcenterroll);

%wishbone
%
%   X = wishbone(ruarmyc,ruarmzc,rlarmyc,rlarmzc,r2,r3,r4,r5,z,roll)
%   gives back a datavector containing locations and angles of a
%   double wishbone suspension.  
%
%   The input parameters are:
%      (ruarmyc,ruarmzc) - (y,z) location of upper control arm inner
%                          attachment point relative to car center
%      (rlarmyc,rlarmzc) - (y,z) location of lower control arm inner
%                          attachment point relative to car center
%      r2   - Lower control arm length
%      r3   - Distance between the outboard ball joints
%      r4   - Upper control arm length
%      r5   - Distance from lower ball joint to the tire contact point
%      z    - Height of (arbitrary) reference point on car
%      roll - Roll of body about reference point
%
%   Locations of the control arms are taken relative to a point used
%   as the center of the carbody.  This may be defined arbitrarily by
%   the user.
%
%   The output elements (defined according to the suspension drawing)
%   of X are given by:
%      X(1) = z;
%      X(2) = roll;
%      X(3) = y1r;
%      X(4) = z1r;
%      X(5) = y2r;
%      X(6) = z2r;
%      X(7) = y3r;
%      X(8) = z3r;
%      X(9) = y4r;
%      X(10) = z4r;
%      X(11) = ytirer;
%      X(12) = theta1r;
%      X(13) = theta2r;
%      X(14) = theta3r;
%      X(15) = theta4r;
%      X(16) = y1l;
%      X(17) = z1l;
%      X(18) = y2l;
%      X(19) = z2l;
%      X(20) = y3l;
%      X(21) = z3l;
%      X(22) = y4l;
%      X(23) = z4l;
%      X(24) = ytirel;
%      X(25) = theta1l;
%      X(26) = theta2l;
%      X(27) = theta3l;
%      X(28) = theta4l;
%
%   The solution of the kinematics assumes that the line from the 
%   upper outer control arm ball joint to the tire contract patch
%   is a rigid link.  The y-coordinate of the car center is taken
%   to be zero.  All dimensions are in centimeters.

% Version 1.1
% Chris Gerdes 
% Dynamic Design Lab
% Stanford University
% 4/16/2002
% Function for use with ME106/227

% First, calculate the positions of the control arm points
% connected to the carbody in light of the given height and
% roll angle.  Then get the values of r1 and theta1.

y1r = rlarmyc*cos(carcenterroll)-rlarmzc*sin(carcenterroll);
z1r = rlarmzc*cos(carcenterroll)+rlarmyc*sin(carcenterroll)+zcarcenter;
y2r = ruarmyc*cos(carcenterroll)-ruarmzc*sin(carcenterroll);
z2r = ruarmzc*cos(carcenterroll)+ruarmyc*sin(carcenterroll)+zcarcenter;
r1 = sqrt((y1r-y2r)^2+(z1r-z2r)^2);
theta1r = atan((y1r-y2r)/(z2r-z1r));

y1l = -rlarmyc*cos(carcenterroll)-rlarmzc*sin(carcenterroll);
z1l = rlarmzc*cos(carcenterroll)-rlarmyc*sin(carcenterroll)+zcarcenter;
y2l = -ruarmyc*cos(carcenterroll)-ruarmzc*sin(carcenterroll);
z2l = ruarmzc*cos(carcenterroll)-ruarmyc*sin(carcenterroll)+zcarcenter;
theta1l = atan((y1l-y2l)/(z2l-z1l));

% With this information, solve for the positions of the right
% suspension linkage with the constraint that the tire contact
% point must lie on the ground.

functozero=inline('(P1 + P2*cos(x) - P3*cos(fourbart(x,P4,P5,P6,P7,P8,P9)))',9);

warning off;
theta2r=fzero(functozero,pi/2,[],z1r,r2,r5,theta1r,r1,r2,r3,r4,1);
warning on;

theta3r=fourbart(theta2r,theta1r,r1,r2,r3,r4,1);
theta4r=fourbart4(theta2r,theta1r,r1,r2,r3,r4,1);

% Calculate outboard control arm points from this information.

y3r = y2r - r4*sin(theta4r);
z3r = z2r + r4*cos(theta4r);
y4r = y1r - r2*sin(theta2r);
z4r = z1r + r2*cos(theta2r);

% Calculate the tire location.

ytirer = (y2r - r4*sin(theta4r) + (r3+r5)*sin(theta3r));

% Now repeat all of this for the left suspension.

functozero=inline('(P1 - P2*cos(x) - P3*cos(fourbart(x,P4,P5,P6,P7,P8,P9)))',9);

warning off;
theta4l=fzero(functozero,pi/2,[],z2l,r4,(r3+r5),theta1l,r1,r4,r3,r2,1);
warning on;

theta3l=fourbart(theta4l,theta1l,r1,r4,r3,r2,1);
theta2l=fourbart4(theta4l,theta1l,r1,r4,r3,r2,1);

ytirel = (y2l + r4*sin(theta4l) + (r3+r5)*sin(theta3l));

y3l = y2l + r4*sin(theta4l);
z3l = z2l - r4*cos(theta4l);
y4l = y1l + r2*sin(theta2l);
z4l = z1l - r2*cos(theta2l);

% Finally load up the output vector with all of these values.

datavector(1) = zcarcenter;
datavector(2) = carcenterroll;
datavector(3) = y1r;
datavector(4) = z1r;
datavector(5) = y2r;
datavector(6) = z2r;
datavector(7) = y3r;
datavector(8) = z3r;
datavector(9) = y4r;
datavector(10) = z4r;
datavector(11) = ytirer;
datavector(12) = theta1r;
datavector(13) = theta2r;
datavector(14) = theta3r;
datavector(15) = theta4r;
datavector(16) = y1l;
datavector(17) = z1l;
datavector(18) = y2l;
datavector(19) = z2l;
datavector(20) = y3l;
datavector(21) = z3l;
datavector(22) = y4l;
datavector(23) = z4l;
datavector(24) = ytirel;
datavector(25) = theta1l;
datavector(26) = theta2l;
datavector(27) = theta3l;
datavector(28) = theta4l;