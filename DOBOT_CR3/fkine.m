function T = fkine(S,M,q)
% FKINE Calculates the forward kinematics using the product of exponentials
% formula.
%
% Inputs: S - 6xn representing the screw axes of the robot
%         M - homogeneous transformation representing the home pose
%         q - joint values
%
% Output: T - homogeneous transformation representing the robot pose
%
% RBE 501 - Robot Dynamics - Spring 2021
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 01/29/2020

    T = eye(4);
    for ii = 1 : length(q); T = T * twist2ht(S(:,ii),q(ii)); end
    T = T * M;
end