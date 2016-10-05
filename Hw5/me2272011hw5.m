function [h_rc,y_rc] = me2272011hw5(roll_angle,plot_on);
% ME 227 
% Spring 2011
% Script for Assignment #5 
%
% Sample script for calculating the geometry of a double
% wishbone suspension as the vehicle rolls and plotting
% these results.  
%
% This script calls wishbone.m and draw_wishbone.m which, 
% in turn, call fourbart.m and fourbart4.m.  These two
% subroutines handle the four-bar linkage solution for the
% suspension geometry.  FindIntersect is also used to find the
% intersections of lines in the suspension schematic.
%
% [h_rc,y_rc] = me2272010hw5(roll_angle,plot_on)
%
%   roll_angle - the roll angle in degrees
%   plot_on    - plotting flag (1 = yes, 0 = no)
%   h_rc       - left and right roll center heights (2-element vector)
%   y_rc       - left and right roll center lateral positions (2-element
%                vector); this moves slightly to account for the rolling of
%                the vehicle body
%
% When plotting, the left RC is a circle and the right RC is an x.



% The parameter values given below are the approximate
% values for a 1999 Mercedes E320.

% Step 1 - Enter the coordinates of the right side suspension
% link points on the car relative to the car center point 

% y and z values for upper control arm attachment point
ruarmyc = -45;
ruarmzc = 22;

% y and z values for lower control arm attachment point
rlarmyc = -33;
rlarmzc = -22;

% Step 2 - Enter the lengths of the suspension links

% Lower control arm
r2 = 39;

% Upper control arm
r4 = 19;

% Distance between the outboard ball joints
r3 = 50.6;

% Distance from lower ball joint to the tire contact point
r5 = 16.6;

% Step 3 - Enter the following values for plotting purposes

% Height and width of box used to represent car body
hcarbox = 50;
wcarbox = 100;

% Angle between tire vertical axis and kingpin axis
tirekpanglel = -9;
tirekpangler = 9;

% Scrub radius
scrubr = 2;

% Height and width of parallelogram used to represent tire
tireh = 61;
tirew = 21.5;
    
% Step 4 - Enter the height of the car center point and 
% the roll about that point then solve and plot  
carcenterroll = roll_angle*pi/180;
zcarcenter = 44;

% Solve for kinematics of the suspension
datavector = wishbone(ruarmyc,ruarmzc,rlarmyc,rlarmzc,r2,r3,r4,r5,zcarcenter,carcenterroll);

% If plotting enabled, plot the suspension properties
if plot_on == 1
    draw_wishbone(datavector,hcarbox,wcarbox,tireh,tirew,scrubr,tirekpangler,tirekpanglel);
    hold;
end

% Unload useful information from the input vector a la Chris Gerdes'
% draw_wishbone function

% Suspension Points
y1r = datavector(3);
z1r = datavector(4);
y2r = datavector(5);
z2r = datavector(6);
y3r = datavector(7);
z3r = datavector(8);
y4r = datavector(9);
z4r = datavector(10);
y1l = datavector(16);
z1l = datavector(17);
y2l = datavector(18);
z2l = datavector(19);
y3l = datavector(20);
z3l = datavector(21);
y4l = datavector(22);
z4l = datavector(23);

% Calculate the instantaneous centers from the suspension links
ICl = FindIntersect([y1l z1l],[y4l z4l],[y2l z2l],[y3l z3l]);
ICr = FindIntersect([y1r z1r],[y4r z4r],[y2r z2r],[y3r z3r]);

% Find the steer axis intersection with the ground
ytirel = datavector(24);
ytirer = datavector(11);
ztirel = 0;
ztirer = 0;

% Calculate the roll center from the lines from the tire contact points to
% the instantaneous centers intersected with vertical lines through the CG
[RCl] = FindIntersect([ytirel-scrubr ztirel],ICl,[zcarcenter*sin(carcenterroll) 0],[0 zcarcenter]);
[RCr] = FindIntersect([ytirer+scrubr ztirer],ICr,[zcarcenter*sin(carcenterroll) 0],[0 zcarcenter]);

% Plot the roll centers, then label and reverse the axes
if plot_on == 1
    plot(RCl(1),RCl(2),'ro');
    plot(RCr(1),RCr(2),'rx');
    xlabel('y (cm)');
    ylabel('z (cm)');
    set(gca,'XDir','reverse'); % this reverses the y-axis, making the left wheel on the left
    % After you have finished with this, release the plot
    hold;  
end;

% Put the hrc values into the output variables
h_rc = [RCl(2) ; RCr(2)];  % roll center heights
y_rc = [RCl(1) ; RCr(1)];  % roll center lateral position (moves slightly due to rolling of vehicle body)

return;