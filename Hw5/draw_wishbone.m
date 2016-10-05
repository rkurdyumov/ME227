function [] = draw_wishbone(datavector,hcarbox,wcarbox,tireh,tirew,scrubr,kptirer,kptirel)

%draw_wishbone
%
%   draw_wishbone(X,hcarbox,wcarbox,tireh,tirew,scrubr,kptirer,kptirel)
%   where X = wishbone(ruarmyc,ruarmzc,rlarmyc,rlarmzc,r2,r3,r4,r5,z,roll)
%   draws a car with a double wishbone suspension geometry.
%
%   The input parameters are:
%      X       - Datavector from the call to the kinematic solver wishbone.m
%      hcarbox - Height of rectangle drawn to represent the car body 
%      wcarbox - Width of rectangle drawn to represent the car body 
%      tireh   - Height of parallelogram drawn to represent the tire
%      tirew   - Width of parallelogram drawn to represent the tire
%      scrubr  - Scrub radius of the vehicle
%      kptirer - Angle between right kingpin axis and right tire vertical axis
%      kptirel - Angle between left kingpin axis and left tire vertical axis
%
%   All dimensions are in centimeters and the drawing illustrates a
%   head-on view of the car (right side of the car is on the left of
%   the screen).  
%

% Version 1.1
% Chris Gerdes 
% Dynamic Design Lab
% Stanford University
% 4/16/2002
% Function for use with ME106/227

% Unload useful information from the input vector

zcarcenter = datavector(1);
carcenterroll = datavector(2);
y1r = datavector(3);
z1r = datavector(4);
y2r = datavector(5);
z2r = datavector(6);
y3r = datavector(7);
z3r = datavector(8);
y4r = datavector(9);
z4r = datavector(10);
ytirer = datavector(11);
%theta1r = datavector(12);  Not needed in this function
%theta2r = datavector(13);  Not needed in this function
theta3r = datavector(14);
%theta4r = datavector(15);  Not needed in this function
y1l = datavector(16);
z1l = datavector(17);
y2l = datavector(18);
z2l = datavector(19);
y3l = datavector(20);
z3l = datavector(21);
y4l = datavector(22);
z4l = datavector(23);
ytirel = datavector(24);
%theta1l = datavector(25);  Not needed in this function
%theta2l = datavector(26);  Not needed in this function
theta3l = datavector(27);
%theta4l = datavector(28);  Not needed in this function

% Calculate the tire inclination angles 

incll = theta3l + kptirel*(pi/180);
inclr = theta3r + kptirer*(pi/180);

% Calculate coordinates for the rectangle representing the car

yc1 = (wcarbox/2)*cos(carcenterroll)+(hcarbox/2)*sin(carcenterroll);
zc1 = zcarcenter - (hcarbox/2)*cos(carcenterroll)+(wcarbox/2)*sin(carcenterroll);
yc2 = (wcarbox/2)*cos(carcenterroll)-(hcarbox/2)*sin(carcenterroll);
zc2 = zcarcenter + (hcarbox/2)*cos(carcenterroll)+(wcarbox/2)*sin(carcenterroll);
yc3 = -(wcarbox/2)*cos(carcenterroll)-(hcarbox/2)*sin(carcenterroll);
zc3 = zcarcenter + (hcarbox/2)*cos(carcenterroll)-(wcarbox/2)*sin(carcenterroll);
yc4 = -(wcarbox/2)*cos(carcenterroll)+(hcarbox/2)*sin(carcenterroll);
zc4 = zcarcenter - (hcarbox/2)*cos(carcenterroll)-(wcarbox/2)*sin(carcenterroll);

% Set up the axes to be equal and square (so rectangles look 
% like rectangles) then draw the car body.

figure(1);
plot([yc1 yc2 yc3 yc4 yc1],[zc1 zc2 zc3 zc4 zc1]);
axis square;
axis equal;
hold; 

% Plot all of the suspension joints

plot(y1r,z1r,'o');
plot(y2r,z2r,'o');
plot(y1l,z1l,'o');
plot(y2l,z2l,'o');

plot(y3r,z3r,'o');
plot(y4r,z4r,'o');
plot(y3l,z3l,'o');
plot(y4l,z4l,'o');

% Draw the suspension links

plot([y1r y4r],[z1r z4r]);
plot([y2r y3r],[z2r z3r]);
plot([y1l y4l],[z1l z4l]);
plot([y2l y3l],[z2l z3l]);

% Draw in the kingpin axis

plot([y3r ytirer],[z3r 0],'--');
plot([y3l ytirel],[z3l 0],'--');

% Draw the tires

plot([(ytirer-scrubr-(tirew/2)) (ytirer + (tirew/2) - scrubr)], [0 0],'k');
plot([(ytirer + (tirew/2) - scrubr) (ytirer + (tirew/2) - scrubr - tireh*sin(inclr))],[0 tireh*cos(inclr)],'k');
plot([(ytirer-scrubr-(tirew/2)) (ytirer-scrubr-(tirew/2) - tireh*sin(inclr))], [0 tireh*cos(inclr)],'k');
plot([(ytirer + (tirew/2) - scrubr - tireh*sin(inclr)) (ytirer-scrubr-(tirew/2) - tireh*sin(inclr))], [tireh*cos(inclr) tireh*cos(inclr)],'k');

plot([(ytirel+scrubr+(tirew/2)) (ytirel - (tirew/2) + scrubr)], [0 0],'k');
plot([(ytirel + (tirew/2) + scrubr) (ytirel + (tirew/2) + scrubr - tireh*sin(incll))],[0 tireh*cos(incll)],'k');
plot([(ytirel+scrubr-(tirew/2)) (ytirel + scrubr - (tirew/2) - tireh*sin(incll))], [0 tireh*cos(incll)],'k');
plot([(ytirel + (tirew/2) + scrubr - tireh*sin(incll)) (ytirel + scrubr-(tirew/2) - tireh*sin(incll))], [tireh*cos(incll) tireh*cos(incll)],'k');

% Release the plot

hold;