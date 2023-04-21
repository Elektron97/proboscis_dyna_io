%%%%%% Solve Equations %%%%%%%
optimal_setup
load("euler_equation.mat")
load("equality_constraint.mat")

%% Better Readbility
optimal_equations = simplify(subs([euler_eq; f_xi], dot_w(1:end-na), w_dot_sym));

%% Try to solve with dsolve
% dsolve([euler_eq; f_xi])