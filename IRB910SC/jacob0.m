function J = jacob0(S,q)
% JACOB0: Calculates the space jacobian of a robotic arm
%
% Inputs: S - screw axes of the robot
%         q - joint values
%
% Output: J - Jacobian expressed in the space frame
    T = eye(4);
    [row,col] = size(S);
    for i = 1:col
        T = T*twist2ht(S(:,i),q(i));
        J(:,i) = adjoint(S(:,i), T);
    end
end