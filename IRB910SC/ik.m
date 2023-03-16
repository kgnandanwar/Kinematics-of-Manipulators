% RBE 501 - Robot Dynamics - Spring 2023
% Midterm Exam
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/21/2023

% Name: Kunal Nandanwar
% ID: 471212488

clear, clc, close all
% addpath('utils');

%% First, execute poe.m to load the S and M matrices
poe
close all

%% Generate and display the path that the robot has to trace
nPts = 10;
path = [0.2 * ones(1,nPts);
        linspace(-0.45, 0.45, 10);
        -0.15 * ones(1,nPts)];

path(3,2:end-1) = path(3,2:end-1) + 0.10;

robot.plot(zeros(1,n)), hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
drawnow;

%% Solve the inverse kinematics
% Set the current joint variables
currentQ = zeros(1,n);

% Create a vector where we are going to store all the solutions for the IK
qList = zeros(size(path,2), n);

% *** YOUR CODE HERE ***
% error('The Inverse Kinematics has not been implemented yet.') % *** COMMENT THIS OUT ***

% Calculate the coordinates representing the home pose
currentXYZ = M(1:3,4);

for ii = 1 : nPts
     robot.plot(currentQ);

     while norm(path(:,ii) - currentXYZ) > 1e-3

         Ja = jacoba(S,M,currentQ); % Function call for analytical Jacobian

         lambda = 0.1;
         deltaQ = Ja' * pinv(Ja*Ja' + lambda^2 * eye(3)) * (path(:,ii) - currentXYZ); % Implemented IK using DLS
         
         currentQ = currentQ + deltaQ'; % Updating the currentQ

         T = fkine(S,M,currentQ);
         currentXYZ = T(1:3,4);

     end
     qList(ii,:) = currentQ; % Appending the values of joint variables in the qList to plot it
end


%% Carry out the pick-and-place task ten times
close all
robot.plot(repmat([qList; flip(qList)], 10, 1), ...
          'trail', {'r', 'LineWidth', 5});