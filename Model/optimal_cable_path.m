%%%%%%%%%%% Optimization Problem %%%%%%%%%%%%%
optimal_setup

% Cost Functional
g = (1/2)*(xi'*W_xi*xi + D'*W_d*D + tau'*W_tau*tau);

%% Constraint
B_tau = actuationMatrix(xi, actuation_path, X);

% f_xi == 0
f_xi = inv(stiff_matrix)*B_tau*tau + xi0 - xi;
save("equality_constraint.mat", "f_xi");

%% Euler Equation
g_a = g + lambda'*(f_xi);

diff_w = simplify(gradient(g_a, w));

% Different. respect to dot_w
% Subst. symbolic variables
g_a_sym = simplify(subs(g_a, dot_w(1:end-na), w_dot_sym));
diff_dotw = simplify(gradient(g_a_sym, w_dot_sym));

% Re-subs
diff_dotw = subs(diff_dotw, w_dot_sym, dot_w(1:end-na));

% Compute Equation
euler_eq = simplify(diff_w - diff([diff_dotw; zeros(na, 1)], X) == zeros(length(w), 1));

%% Save Euler Equation
save("euler_equation.mat", "euler_eq");