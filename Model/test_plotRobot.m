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

%% Actuator Design: Soluzione 1

%% Actuator Design: Soluzione 2
% Robotics Actuators: 8 cables (2 proposta)
na = 8;

% Long. Cables
actuation_path(:, 1) = [0; geom_robot.b0 - ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
actuation_path(:, 2) = [0; -geom_robot.b0 + ((geom_robot.b0 - geom_robot.b)/geom_robot.L)*X; X];
actuation_path(:, 3) = [geom_robot.a0 - ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X; 0; X];
actuation_path(:, 4) = [-geom_robot.a0 + ((geom_robot.a0 - geom_robot.a)/geom_robot.L)*X; 0; X];

% Oblique Cables
actuation_path(:, 5) = ellipticHelix(geom_robot, X, -pi, 0);
actuation_path(:, 6) = ellipticHelix(geom_robot, X, pi, 0);
actuation_path(:, 7) = ellipticHelix(geom_robot, X, -pi, pi);
actuation_path(:, 8) = ellipticHelix(geom_robot, X, pi, pi);

cable_class = ["l", "l", "l", "l", "ocw", "occw", "ocw", "occw"];

%% Plot Robot
plotRobotq0(geom_robot, actuation_path, 100, cable_class, true)

%% Evaluation of Strain Modes
% [xi, ~, ~] = trivialGVS(zeros(6, 1), eye(6), actuation_path, geom_robot.L, ones(na, 1));
% prettyStrainPlot(xi)

%% Util Functions
function points = ellipticHelix(geom_struct, X, theta, phase)
    points = [geom_struct.ax*cos(theta*(X/geom_struct.L) + phase); geom_struct.bx*sin(theta*(X/geom_struct.L) + phase); X];
end