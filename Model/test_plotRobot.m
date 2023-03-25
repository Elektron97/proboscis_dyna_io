%%%%%%%%%%% testing Plot Functions
clear all
close all
clc

%% Create geometrical Properties
geom_robot.a0 = 210e-3; %[m]
geom_robot.b0 = 120e-3; %[m]

geom_robot.a = 70e-3; %[m]
geom_robot.b = 40e-3; %[m]

geom_robot.L = 700e-3; %[m]

geom_robot.ax = @(X) geom_robot.a0 - ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X;
geom_robot.bx = @(X) geom_robot.b0 - ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X;

syms X real

%% Actuator Design
% Robotics Actuators: 8 cables (2 proposta)
% na = 8;
% actuation_path = sym(zeros(3, na));
% Long. Cables
actuation_path(:, 1) = [0; geom_robot.b0 - ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
actuation_path(:, 2) = [0; -geom_robot.b0 + ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
actuation_path(:, 3) = [geom_robot.a0 - ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X; 0; X];
actuation_path(:, 4) = [-geom_robot.a0 + ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X; 0; X];

% Oblique Cables
% actuation_path(:, 5) = [0; geom_robot.b0 - ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];


% % Test Helicoidals
% actuation_path(:, 1) = [geom_robot.ax*cos(2*pi*X); geom_robot.bx*sin(2*pi*X); X];
% actuation_path(:, 2) = [geom_robot.ax*cos(2*pi*X + 2*pi/3); geom_robot.bx*sin(2*pi*X + 2*pi/3); X];
% actuation_path(:, 3) = [geom_robot.ax*cos(2*pi*X - 2*pi/3); geom_robot.bx*sin(2*pi*X - 2*pi/3); X];

plotRobotq0(geom_robot, actuation_path, 1000)

%% Evaluation of Strain Modes
[xi, ~] = trivialGVS(zeros(6, 1), eye(6), actuation_path, 1, [1, 1, 1, 1]');
prettyStrainPlot(xi)