% RBE 501 - Robot Dynamics - Fall 2021
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***<YOUR NAME HERE>***
clear, clc, close all
addpath('utils');

% First, execute poe.m to load the S and M matrices
poe
close all

% Generate and display the path that the robot has to trace
t = linspace(-pi, pi, 36);
x = 0.3  * ones(1,36);
%a = 0.4;
y = (10 * (sin(t)).^3)./60;
z = (13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t))./60 + 0.3;
path = [x; y; z];

scatter3(path(1,:), path(2,:), path(3,:), 'filled');


% Convert Cartesian coordinates into twists
targetPose = zeros(6,size(path,2)); % each column of this matrix is a target pose represented by a twist

for ii = 1 : size(path,2)
    % First calculate the homogeneous transformation matrix representing
    % each target pose
    R = [0 0 -1; 0 1 0; 1 0 0]';
    T = [R path(:,ii); 
         0 0 0 1];
     
    % Then perform the matrix logarithm operation to convert transformation
    % matrices into 4x4 elements of se(3)
    t = MatrixLog6(T);
    
    % Finally, "unpack" this matrix (i.e., perform the inverse of the
    % bracket operator)
    targetPose(:,ii) = [t(3,2) t(1,3) t(2,1) t(1:3,4)']';
end


%% Calculate the inverse kinematics 
% Starting from the home configuration, iteratively move the robot to each
% of the poses in the `targetPose` matrix

currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,n);
robot.plot(currentQ);
title('Inverse Kinematics Test');

qList = zeros(size(path,2), n);

for ii = 1 : size(path,2)
    while norm(targetPose(:,ii) - currentPose) > 1e-3
        J = jacob0(S,currentQ);
        
        % Levenberg–Marquardt algorithm
        lambda = 0.1;
        deltaQ = J' * pinv(J*J' + lambda^2 * eye(6))*(targetPose(:,ii) - currentPose);
                
        currentQ = currentQ + deltaQ';
        
        %Update the current pose
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        
        currentPose = [currentPose(3,2) ...
            currentPose(1,3) ...
            currentPose(2,1) ...
            currentPose(1:3,4)']';
        
%         % Display the robot
%         robot.plot(currentQ);
%         drawnow;
    end
    
    qList(ii,:) = currentQ;
end

close all
robot.plot(qList, 'trail', {'r', 'LineWidth', 5});
fprintf('\nIK solved successfully.\n');