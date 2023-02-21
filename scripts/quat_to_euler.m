function [heading,elevation,roll] = quat_to_euler(q)
%QUAT_TO_EULER q =[x,y,z,w]
x=q(1);
y=q(2);
z=q(3);
w=q(4);
heading   = atan2(2*(w*z+x*y),1-2*(y^2 + z^2));
elevation = asin(2*(w*y-z*x));
roll      = atan2(2*(w*x+y*z),1-2*(x^2+y^2));
end

