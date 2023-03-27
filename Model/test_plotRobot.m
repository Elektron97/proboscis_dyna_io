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

% graphical parameters
n_points = 100;

% Symbolic Variable
syms X real

%% Show only Robot Shape
% actuation_path(:, 1) = sym(zeros(3, 1));
% plotRobotq0(geom_robot, actuation_path, n_points)

%% Actuator Design: Soluzione 1
% % Bio-inspired Actuators: 7 cables (1 proposta)
% na = 7;
% 
% %%% Long. Cables %%%
% actuation_path(:, 1) = [0; geom_robot.b0 - ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
% 
% % 30 degrees long cable
% p1 = [-geom_robot.a0*cos(pi/6); geom_robot.b0*sin(pi/6); 0];
% p2 = [-geom_robot.a*cos(pi/6); geom_robot.b*sin(pi/6); geom_robot.L];
% actuation_path(:, 2) = p1 + X*((p2 - p1)/geom_robot.L);
% 
% % 150 degrees long cable
% p3 = [geom_robot.a0*cos(pi/6); geom_robot.b0*sin(pi/6); 0];
% p4 = [geom_robot.a*cos(pi/6); geom_robot.b*sin(pi/6); geom_robot.L];
% actuation_path(:, 3) = p3 + X*((p4 - p3)/geom_robot.L);
% 
% %%% Curv. Cables %%%
% actuation_path(:, 4) = ellipticHelix(geom_robot, X, -pi/2, 0);
% actuation_path(:, 5) = ellipticHelix(geom_robot, X, -pi/2, -pi/2);
% actuation_path(:, 6) = ellipticHelix(geom_robot, X, pi/2, pi);
% actuation_path(:, 7) = ellipticHelix(geom_robot, X, pi/2, 1.5*pi);
% 
% cable_class = ["l", "l", "l", "ocw", "ocw", "occw", "occw"];
% 
% %% Plot Robot
% plotRobotq0(geom_robot, actuation_path, n_points, cable_class, true)

%% Actuator Design: Soluzione 2
% % Robotics Actuators: 8 cables (2 proposta)
% na = 8;
% 
% % Long. Cables
% actuation_path(:, 1) = [0; geom_robot.b0 - ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
% actuation_path(:, 2) = [0; -geom_robot.b0 + ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
% actuation_path(:, 3) = [geom_robot.a0 - ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X; 0; X];
% actuation_path(:, 4) = [-geom_robot.a0 + ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X; 0; X];
% 
% % Oblique Cables
% actuation_path(:, 5) = ellipticHelix(geom_robot, X, -pi, 0);
% actuation_path(:, 6) = ellipticHelix(geom_robot, X, pi, 0);
% actuation_path(:, 7) = ellipticHelix(geom_robot, X, -pi, pi);
% actuation_path(:, 8) = ellipticHelix(geom_robot, X, pi, pi);
% 
% cable_class = ["l", "l", "l", "l", "ocw", "occw", "ocw", "occw"];
% 
% %% Plot Robot
% plotRobotq0(geom_robot, actuation_path, n_points, cable_class, true)

%% Actuator Design: Soluzione 3
% % Hybrid Solution: 8 cables (3 proposta)
% na = 8;
% 
% % Long. Cables
% actuation_path(:, 1) = [0; geom_robot.b0 - ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
% actuation_path(:, 2) = [0; -geom_robot.b0 + ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
% actuation_path(:, 3) = [geom_robot.a0 - ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X; 0; X];
% actuation_path(:, 4) = [-geom_robot.a0 + ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X; 0; X];
% 
% % Curv. Cables
% actuation_path(:, 5) = ellipticHelix(geom_robot, X, -pi/2, 0);
% actuation_path(:, 6) = ellipticHelix(geom_robot, X, pi/2, 0);
% actuation_path(:, 7) = ellipticHelix(geom_robot, X, -pi/2, pi);
% actuation_path(:, 8) = ellipticHelix(geom_robot, X, pi/2, pi);
% 
% cable_class = ["l", "l", "l", "l", "ocw", "occw", "ocw", "occw"];
% 
% %% Plot Robot
% plotRobotq0(geom_robot, actuation_path, n_points, cable_class, true)

%% Evaluation of Strain Modes
% [xi, ~, ~] = trivialGVS(zeros(6, 1), eye(6), actuation_path, geom_robot.L, ones(na, 1));
% prettyStrainPlot(xi)

%% Util Functions
function points = ellipticHelix(geom_struct, X, theta, phase)
    points = [geom_struct.ax*cos(theta*(X/geom_struct.L) + phase); geom_struct.bx*sin(theta*(X/geom_struct.L) + phase); X];
end