%%%%%%%%%%%% Actuation Path: Testing and Simulation %%%%%%%%%%%%%%%%
clear all
close all
clc

%% Testing trivialGVS Function
syms X real

% % Linear Path
% actuation_path = [0; 1+X; 1+X];
% [xi, Bq] = trivialGVS(zeros(6, 1), eye(6), actuation_path, 1, 1);

% % Helicoidal Path
% actuation_path = [0; 1*cos(2*pi*X); 1*sin(2*pi*X)];
% [xi, Bq] = trivialGVS(zeros(6, 1), eye(6), actuation_path, 1, 1);

% Multi-Linear Path
% actuation_path = [0, 0, 0; 1+X, 3+2*X, -1-X; 1+X, 3+2*X, -1-X];
% [xi, Bq] = trivialGVS(zeros(6, 1), eye(6), actuation_path, 1, [1 1 0]');

% Multi-Helicoidal Path
actuation_path = 1*[X, X, X; cos(2*pi*X), cos(2*pi*X + 2*pi/3), cos(2*pi*X + 4*pi/3); sin(2*pi*X), sin(2*pi*X + 2*pi/3), sin(2*pi*X + 4*pi/3)];
[xi, ~] = trivialGVS(zeros(6, 1), eye(6), actuation_path, 1, [1, 1, 1]');

%% Normalization for a better Visualization
xi_norm = xi/norm(xi);
prettyStrainPlot(xi_norm)