%%%%%%%%  Analysis of B_tau %%%%%%%
addpath("autogen_functions");

xi0 = [zeros(5, 1); 1];

%% Evaluate rank for all the length of the robot
integration_step = 0.001;
arc_length = linspace(0, geom_robot.L, 1/integration_step);

for i = 1:length(arc_length)
    current_Btau = B_tau_sol1(xi0, arc_length(i));
    rank_X(i) = rank(current_Btau);
    null_space(:, i) = null(current_Btau);
end

figure
hold on
for j = 1:na
    plot(arc_length, abs(null_space(j, :)))
end
hold off
grid on
title("Null Space of B_{\tau}")
xlabel("Arc Length [m]")
ylabel("Tension of Cables [N]")
legend("Cable 1", "Cable 2", "Cable 3", "Cable 4", "Cable 5", "Cable 6", "Cable 7")


%% Numerical Integration to Obtain B
B_handle = @(s) B_tau_sol1(xi0, s);
B = integral(B_handle, 0, geom_robot.L, 'ArrayValued',true);

%% Clear
clear xi0 arc_length