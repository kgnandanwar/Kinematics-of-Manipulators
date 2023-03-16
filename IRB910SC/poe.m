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

n = 4;   % degrees of freedom
plotOn = true;

%% Create the manipulator and display it in its home configuration
[robot, jointLimits] = make_robot();

q = zeros(1,n);
robot.teach(zeros(1,n));

%% Question 1 - Calculate the screw axes
% S = ...
S = [0 0 1 0 0 0;
     0 0 1 0 -0.300 0;
     0 0 0 0 0 -1;
     0 0 -1 0 0.550 0]';

%% Question 2 - Calculate the home configuration
% M = ...
R = [1 0 0;0 -1 0;0 0 -1];
p = [0.55 0 0]';
M = [R p; 0 0 0 1];

% Random samples of joine parameter values are created and are stored in the matrix q
nTests = 20;
for i = 1:nTests
    q(i,:) = [  jointLimits(1,1) + (jointLimits(1,2) - jointLimits(1,1)) * rand(), ...
                jointLimits(2,1) + (jointLimits(2,2) - jointLimits(2,1)) * rand(), ...
                jointLimits(3,1) + (jointLimits(3,2) - jointLimits(3,1)) * rand(), ...
                jointLimits(4,1) + (jointLimits(4,2) - jointLimits(4,1)) * rand()];
                
              
end

%% Calculating Forward Kinematics for certain fixed values of Q

    fprintf('---------------------Forward Kinematics Test ---------------------\n');
    fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
    fprintf('Progress: ');
    nbytes = fprintf('0%%'); 
     
    % Test the forward kinematics for 100 random sets of joint variables
    for i = 1 : nTests
        fprintf(repmat('\b',1,nbytes));
        nbytes = fprintf('%0.f%%', ceil(i/nTests*100));
        
        % Calculate the forward kinematics
        T = fkine(S,M,q(i,:));
        
        if plotOn
            robot.teach(q(i,:));
            title('Forward Kinematics Test');
        end
       
    %{ 
        Checking if the Transformation matrix obtained throught the user-defined fkine function is quivalent
        to the predefined Tranformation matrix using D-H parameters.The
        tolerance of error is set to 1^-10.
    %}     
        assert(all(all(abs(double(robot.fkine(q(i,:))) - T) < 1e-10)));
    end
 
    fprintf('\nTest passed successfully.\n');
