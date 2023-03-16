function T = twist2ht(S,theta)
% TWIST2HT Converts a twist into a homogeneous transformation matrix
%
% Inputs: S - Screw Axis
%         theta - scalar representing the amount of motion
%
% Output: T - Homogeneous Transformation matrix (SE(3))

    omega = [S(1); S(2);S(3)];
    v = [S(4); S(5); S(6)];
    omegahat = [0 -omega(3) omega(2) ; omega(3) 0 -omega(1) ; -omega(2) omega(1) 0 ];
    p = sin(theta);
    I = eye(3);
    q = 1 - cos(theta);
    m = theta - p;
    R = I + p*omegahat + q*(omegahat)^2;
    W = (I*theta + q*omegahat + m*(omegahat)^2)*v;
    T = [R W; 0 0 0 1];

end