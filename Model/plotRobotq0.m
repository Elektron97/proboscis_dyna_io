%%%%%%%%%%% Plot Proboscis Standard Configuration %%%%%%%%%
function plotRobotq0(geom_params, actuation_path, n_points)
%%%%%%% Input %%%%%%%%
% Actuation Path: Symbolic Expression in X
% Geometrical Parameters: Structure with all information

%% Input Processing
L = geom_params.L;
[~, na] = size(actuation_path);
% Convert in Function Handle a Symbolic Expression
actuation_path = matlabFunction(actuation_path, 'vars', {'X'});

axis_scale = 0.1;
%% Cross Sections
% Plot Proximal Cross-Section
[proximal_pc0, ~] = ellipse3D(geom_params.a0, geom_params.b0, ...
                                0, 0, 0, n_points, ...
                                0, 0, 0, 0);
% Plot Distal Cross-Section
[proximal_pc, ~] = ellipse3D(geom_params.a, geom_params.b, ...
                                0, 0, L, n_points, ...
                                0, 0, 0, 0);

%% Actuation Path
arc_values = 0:(L/n_points):L;

for i=1:na
    for j = 1:length(arc_values)
        act_arc = actuation_path(arc_values(j));
        actuation_pc{i}(:, j) = act_arc(:, i);
    end
end

%% Dorsal/Ventral
% x = linspace(-geom_params.a0, geom_params.a0);
% z = linspace(0, L);
% [X, Z] = meshgrid(x, z);
% Y = zeros(size(X, 1)); % Generate z data


%% Body of the Robot
% To Do


%% Create Figure
figure
plot3(proximal_pc0(1, :), proximal_pc0(2, :), proximal_pc0(3, :), 'Color', [0.8500 0.3250 0.0980])
hold on
trplot(eye(4), 'frame', 'Proximal', 'rgb', 'length', axis_scale*L);
plot3(proximal_pc(1, :), proximal_pc(2, :), proximal_pc(3, :), 'Color', [0.8500 0.3250 0.0980])
trplot([eye(3), [0 0 L]'; zeros(1, 3), 1], 'frame', 'Distal', 'rgb', 'length', axis_scale*L);

% Plot Actuators
for i=1:na
    plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0.4470 0.7410])
%     plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0 0])
end
% % Midline Plane
% s = surf(X, Y, Z); % Plot the surface
% s.EdgeColor = 'none';
% s.FaceAlpha = 0.1;
% s.FaceColor = [0.4660 0.6740 0.1880];

view(38, 31)
grid on
xlabel("[m]")
ylabel("[m]")
zlabel("[m]")
title("Strain-less PROBOSCIS Robot")
axis equal
hold off
end