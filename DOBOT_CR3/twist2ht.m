function T = twist2ht(S,theta)
% TWIST2HT Converts a twist into a homogeneous transformation matrix
%
% Inputs: S - 6D twist
%         theta - scalar representing the amount of motion
%
% Output: T - Homogeneous Transformation matrix (SE(3))
%
% RBE 501 - Robot Dynamics - Spring 2021
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 01/29/2020

    omega = S(1:3);
    v     = S(4:6);
    
    if norm(omega) < eps(10)
        T = [eye(3) v*theta; 
             0 0 0 1];
        
    elseif norm(omega) - 1 < eps(10)
        R = axisangle2rot(omega,theta);
        
        omega_ss = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
        T = [R (eye(3)*theta + (1-cos(theta))*omega_ss + (theta - sin(theta))*omega_ss^2)*v;
             0 0 0 1];
    else
        disp('what');
    end
end