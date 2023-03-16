function J = jacob0(S,q)
% JACOB0 Calculates the space jacobian of a robotic arm
%
% Inputs: S - 6xn representing the screw axes of the robot
%         q - joint values
%
% Output: J - 6xn Jacobian expressed in the space frame
%
% RBE 501 - Robot Dynamics - Spring 2021
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 01/29/2020
 
    % Read the number of joints
    n = length(q);

    % Calculate the homogeneous transformations to each joint
    T = zeros(4,4,n-1);
    
    for ii = 1 : n-1
        if ii == 1
            T(:,:,ii) = twist2ht(S(:,ii),q(ii));
        else
            T(:,:,ii) = T(:,:,ii-1) * twist2ht(S(:,ii),q(ii));
        end
    end
    
    % Calculate the Jacobian in the space frame
    J = zeros(6,n);
    J(:,1) = S(:,1); % The first column is equal to the screw axis for the first joint
    
    for ii = 2 : n
        J(:,ii) = adjoint(S(:,ii),T(:,:,ii-1));
    end
end