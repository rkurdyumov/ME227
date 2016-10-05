function theta4 = fourbart4(theta2,theta1,r1,r2,r3,r4,assembly);

r7 = sqrt(r1^2+r2^2-2*r1*r2*cos(theta2-theta1));
Psi=acos((r4^2+r7^2-r3^2)/(2*r4*r7));
    
if (theta2 >= theta1)&(theta2 <= theta1 + pi)
   alpha = acos((r1^2+r7^2-r2^2)/(2*r1*r7));
else
   alpha = -acos((r1^2+r7^2-r2^2)/(2*r1*r7));
end

theta4 = pi - alpha + theta1 - Psi*assembly;
theta3t = asin((r1*sin(theta1)-r2*sin(theta2)+r4*sin(theta4))/r3);
    
if (r4*cos(theta4)+r1*cos(theta1)-r2*cos(theta2) < 0)
   theta3 = pi - theta3t;
else
   theta3 = theta3t;
end
