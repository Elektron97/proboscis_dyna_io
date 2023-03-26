%%%%%%%%%%%%%%%%%%% Trivial Model from GVS approach %%%%%%%%%%%%%%%%%%%%%
% This function implement a numerical resolution of Trivial Model for   %
% Continuum Soft Robots, proposed by Armanin et al. (2020).             %
% Future Steps: Include also symbolic computation, to find an analytic  %
% solutions. However, in the most frequent cases, it's better a num.    %
% solution.                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [finded_xi, Bq, B_tau] = trivialGVS(xi0, stiff_matrix, actuation_path, X_des, tau)
% Input Description:
% xi0:              6D vector
% stiff_matrix:     6x6 matrix, Usually diag([G*Jx, E*Jy, E*Jz, E*A, G*A, G*A]);
% Actuation Path:   Symbolic Expression only in X of a nax3 Matrix (e.g. 1 + 2X, 5*cos(0.5*X))
% X_des:            Desired Value of Arc Length
% tau:              nax1 Vector of actuation intensities

% Output Description:
% xi:               6D Vector that represents the Strain Modes excited by Actuation
% Bq:               6xn Matrix that map the joint variables in to Strain

%% Declare Symbolic Variables
syms X real                         % Arc Length
syms kx ky kz real                  % torsion_x, bending_y, bending_z
syms sigma_x sigma_y sigma_z real   % stretching_x, shear_y, shear_z

xi = [kx; ky; kz; sigma_x; sigma_y; sigma_z];
%% Compute Actuation Matrix
B_tau = actuationMatrix(xi, actuation_path, X, X_des);

%% Compute the Nonlinear Equation
equation = xi == simplify(inv(stiff_matrix)*B_tau*tau) + xi0;

%% Solve it Numerically
solutions = vpasolve(equation, xi);

%% Output
finded_xi = [solutions.kx, solutions.ky, solutions.kz, solutions.sigma_x, solutions.sigma_y, solutions.sigma_z]';
B_tau = subs(B_tau, xi, finded_xi);
Bq = inv(stiff_matrix)*B_tau;
end