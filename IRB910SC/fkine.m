function T = fkine(S,M,q)
% FKINE Calculates the forward kinematics using the product of exponentials
% formula.
%
% Inputs: S - 6xn representing the screw axes of the robot
%         M - homogeneous transformation representing the home pose
%         q - joint values
%
% Output: T - homogeneous transformation representing the robot pose

    [~,col] = size(S);
    T = eye(4);
    
    for i = 1:col
        T = T*twist2ht(S(:,i), q(i));
    end
    T = T*M;
    
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i));
end