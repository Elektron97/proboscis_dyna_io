%%%% Optimal Control Variables and Params %%%%
clear all
close all
clc

addpath("functions");

%% Useful params
na = 7; % Optimization Process with known number of cables
stiff_matrix = eye(6);
xi0 = zeros(6, 1);

%% Define Symbolic Variables
syms X real
syms d(X) [na, 3]
tau = sym('tau_%d', [1, na], 'real')';
syms kappa(X) [3, 1]
syms sigma(X) [3, 1]

% Lagrangian Multipliers
syms lambda(X) [6, 1]

% Symbolic Dot w
syms w_dot_sym(X) [6 + 3*na, 1]
syms w_2dot_sym(X) [6 + 3*na, 1]

% Make them Real
assumeAlso(d(X),'real')
assumeAlso(kappa(X),'real')
assumeAlso(sigma(X),'real')
assumeAlso(lambda(X), 'real')
assumeAlso(w_dot_sym(X),'real')
assumeAlso(w_2dot_sym(X),'real')

actuation_path = d(X)';
D = reshape(actuation_path, 3*na, 1);

xi = [kappa(X); sigma(X)];

w = [xi; D; tau];

% Define differentiated variables
dot_w = sym(zeros(6 + 3*na, 1));
for i=1:length(w)
    dot_w(i) = diff(w(i), X); 
end

% Symbolic Diff. Variables
w_dot_sym = w_dot_sym(X);

lambda = lambda(X);

assumeAlso(dot_w,'real')

%% Cost Functional
% Weight Matrices
W_xi = eye(6);
W_d = eye(3*na);
W_tau = eye(na);