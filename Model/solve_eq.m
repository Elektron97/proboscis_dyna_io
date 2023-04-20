%%%%%% Solve Equations %%%%%%%
optimal_setup
load("euler_equation.mat")
load("equality_constraint.mat")

%% Better Readbility
optimal_equations = subs([euler_eq; f_xi], dot_w(1:end-na), w_dot_sym);

% Select only Algebraic Equations
algebraic_eq = optimal_equations([1:6, 6+na+1:end]);
% Try to solve algebraic equations in terms of d, d', d'', ...
algebraic_sol = solve(algebraic_eq, [xi; tau; lambda])

%% Try to solve with dsolve
% dsolve([euler_eq; f_xi])