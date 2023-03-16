function R = axisangle2rot(omega,theta)
%{
axisangle2rot is used to convert exponential coordinate representation of 
rigid body transformation to its corresponding rotation matrix
Inputs: omega - Axis of rotation ; theta - rotation angle
output: R - Rotation matrix
%}
    k = [[0,-omega(3),omega(2)];[omega(3),0,-omega(1)];[-omega(2),omega(1),0]];
    R = [[1,0,0];[0,1,0];[0,0,1]] + sin(theta)*k + (1 - cos(theta))*(k*k);2;
end