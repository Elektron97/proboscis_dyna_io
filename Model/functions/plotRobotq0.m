%%%%%%%%%%% Plot Proboscis Standard Configuration %%%%%%%%%
function plotRobotq0(geom_params, actuation_path, n_points, cableCat, ort_plot)
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
x = linspace(-geom_params.a0, geom_params.a0);
z = linspace(0, L);
[X, Z] = meshgrid(x, z);
Y = zeros(size(X, 1)); % Generate z data


%% Body of the Robot
% Define limit of parameters
s = linspace(0, geom_params.L, n_points);
theta = linspace(0, 2*pi, n_points);
% Create the grid
[S, THETA] = meshgrid(s, theta);

% Compute Points
X_body = geom_params.ax(S).*cos(THETA);
Y_body = geom_params.bx(S).*sin(THETA);
Z_body = S;

%% Create Figure
f1 = figure;
plot3(proximal_pc0(1, :), proximal_pc0(2, :), proximal_pc0(3, :), 'Color', [0.8500 0.3250 0.0980])
hold on
trplot(eye(4), 'frame', 'Proximal', 'rgb', 'length', axis_scale*L);
plot3(proximal_pc(1, :), proximal_pc(2, :), proximal_pc(3, :), 'Color', [0.8500 0.3250 0.0980])
trplot([eye(3), [0 0 L]'; zeros(1, 3), 1], 'frame', 'Distal', 'rgb', 'length', axis_scale*L);

% Plot Actuators
for i=1:na
    if(nargin > 3)
        % Longitudinal Cable
        if cableCat(i) == "l"
            plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.0)
        % Oblique (or curved) Cable
        elseif cableCat(i) == "o"
            plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2.0)
        % Oblique ClockWise
        elseif cableCat(i) == "ocw"
            plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 2.0)
        % Oblique CounterClockWise
        elseif cableCat(i) == "occw"
            plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 2.0)
        %  Not Classified: Red
        else
            plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [1 0 0], 'LineWidth', 2.0)
        end
    else
        plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0 0], 'LineWidth', 2.0)
    end
end
% % Midline Plane
s = surf(X, Y, Z); % Plot the surface
s.EdgeColor = 'none';
s.FaceAlpha = 0.3;
s.FaceColor = [0.4660 0.6740 0.1880];

text(geom_params.a0, geom_params.b0*0.9, L, "Dorsal");
text(geom_params.a0, -geom_params.b0*0.9, L, "Ventral");

% Body of Robot
s_body = surf(X_body, Y_body, Z_body);
s_body.EdgeColor = 'None';
s_body.FaceAlpha = 0.5;
s_body.FaceColor = [0.8500 0.3250 0.0980];

grid on
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")
title("Stress-less PROBOSCIS Robot")
axis equal
hold off

%% Orthogonal Projection
if((nargin > 4) && ort_plot)
        f2 = figure;
%     %%% Plot Only XY Plane %%%
%     f2 = figure;
%     plot3(proximal_pc0(1, :), proximal_pc0(2, :), proximal_pc0(3, :), 'Color', [0.8500 0.3250 0.0980])
%         hold on
%     %     trplot(eye(4), 'frame', 'Proximal', 'rgb', 'length', axis_scale*L);
%         plot3(proximal_pc(1, :), proximal_pc(2, :), proximal_pc(3, :), 'Color', [0.8500 0.3250 0.0980])
%     %     trplot([eye(3), [0 0 L]'; zeros(1, 3), 1], 'frame', 'Distal', 'rgb', 'length', axis_scale*L);
%         
%         % Plot Actuators
%         for i=1:na
%             if(nargin > 3)
%                 % Longitudinal Cable
%                 if cableCat(i) == "l"
%                     plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.0)
%                 % Oblique (or curved) Cable
%                 elseif cableCat(i) == "o"
%                     plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2.0)
%                 % Oblique ClockWise
%                 elseif cableCat(i) == "ocw"
%                     plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 2.0)
%                 % Oblique CounterClockWise
%                 elseif cableCat(i) == "occw"
%                     plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 2.0)
%                 %  Not Classified: Red
%                 else
%                     plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [1 0 0], 'LineWidth', 2.0)
%                 end
%             else
%                 plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0 0], 'LineWidth', 2.0)
%             end
%         end
%         % % Midline Plane
%         s = surf(X, Y, Z); % Plot the surface
%         s.EdgeColor = 'none';
%         s.FaceAlpha = 0.3;
%         s.FaceColor = [0.4660 0.6740 0.1880];
%         
%         text(geom_params.a0, geom_params.b0*0.9, L, "Dorsal");
%         text(geom_params.a0, -geom_params.b0*0.9, L, "Ventral");
%         
%         % Body of Robot
%         s_body = surf(X_body, Y_body, Z_body);
%         s_body.EdgeColor = 'None';
%         s_body.FaceAlpha = 0.5;
%         s_body.FaceColor = [0.8500 0.3250 0.0980];
%         
%         grid on
%         xlabel("x [m]")
%         ylabel("y [m]")
%         zlabel("z [m]")
%         title("XY Plane")
%         axis equal
%         view(-90, 90) %xy

    %%%%%%%%%%%%%% XY PLANE %%%%%%%%%%%%%
    subplot(1, 3, 1)
    plot3(proximal_pc0(1, :), proximal_pc0(2, :), proximal_pc0(3, :), 'Color', [0.8500 0.3250 0.0980])
    hold on
%     trplot(eye(4), 'frame', 'Proximal', 'rgb', 'length', axis_scale*L);
    plot3(proximal_pc(1, :), proximal_pc(2, :), proximal_pc(3, :), 'Color', [0.8500 0.3250 0.0980])
%     trplot([eye(3), [0 0 L]'; zeros(1, 3), 1], 'frame', 'Distal', 'rgb', 'length', axis_scale*L);
    
    % Plot Actuators
    for i=1:na
        if(nargin > 3)
            % Longitudinal Cable
            if cableCat(i) == "l"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.0)
            % Oblique (or curved) Cable
            elseif cableCat(i) == "o"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2.0)
            % Oblique ClockWise
            elseif cableCat(i) == "ocw"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 2.0)
            % Oblique CounterClockWise
            elseif cableCat(i) == "occw"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 2.0)
            %  Not Classified: Red
            else
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [1 0 0], 'LineWidth', 2.0)
            end
        else
            plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0 0], 'LineWidth', 2.0)
        end
    end
    % % Midline Plane
    s = surf(X, Y, Z); % Plot the surface
    s.EdgeColor = 'none';
    s.FaceAlpha = 0.3;
    s.FaceColor = [0.4660 0.6740 0.1880];
    
    text(geom_params.a0, geom_params.b0*0.9, L, "Dorsal");
    text(geom_params.a0, -geom_params.b0*0.9, L, "Ventral");
    
    % Body of Robot
    s_body = surf(X_body, Y_body, Z_body);
    s_body.EdgeColor = 'None';
    s_body.FaceAlpha = 0.5;
    s_body.FaceColor = [0.8500 0.3250 0.0980];
    
    grid on
    xlabel("x [m]")
    ylabel("y [m]")
    zlabel("z [m]")
    title("XY Plane")
    axis equal
    view(-90, 90) %xy
    
    %%%%%%%%%%%%%% XZ PLANE %%%%%%%%%%%%%
    subplot(1, 3, 2)
    plot3(proximal_pc0(1, :), proximal_pc0(2, :), proximal_pc0(3, :), 'Color', [0.8500 0.3250 0.0980])
    hold on
%     trplot(eye(4), 'frame', 'Proximal', 'rgb', 'length', axis_scale*L);
    plot3(proximal_pc(1, :), proximal_pc(2, :), proximal_pc(3, :), 'Color', [0.8500 0.3250 0.0980])
%     trplot([eye(3), [0 0 L]'; zeros(1, 3), 1], 'frame', 'Distal', 'rgb', 'length', axis_scale*L);
    
    % Plot Actuators
    for i=1:na
        if(nargin > 3)
            % Longitudinal Cable
            if cableCat(i) == "l"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.0)
            % Oblique (or curved) Cable
            elseif cableCat(i) == "o"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2.0)
            % Oblique ClockWise
            elseif cableCat(i) == "ocw"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 2.0)
            % Oblique CounterClockWise
            elseif cableCat(i) == "occw"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 2.0)
            %  Not Classified: Red
            else
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [1 0 0], 'LineWidth', 2.0)
            end
        else
            plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0 0], 'LineWidth', 2.0)
        end
    end
    % % Midline Plane
    s = surf(X, Y, Z); % Plot the surface
    s.EdgeColor = 'none';
    s.FaceAlpha = 0.3;
    s.FaceColor = [0.4660 0.6740 0.1880];
    
%     text(geom_params.a0, geom_params.b0*0.9, L, "Dorsal");
%     text(geom_params.a0, -geom_params.b0*0.9, L, "Ventral");
    
    % Body of Robot
    s_body = surf(X_body, Y_body, Z_body);
    s_body.EdgeColor = 'None';
    s_body.FaceAlpha = 0.5;
    s_body.FaceColor = [0.8500 0.3250 0.0980];
    
    grid on
    xlabel("x [m]")
    ylabel("y [m]")
    zlabel("z [m]")
    title("XZ Plane")
    axis equal
    view(0, 0) %xz
    
    %%%%%%%%%%%%%% YZ PLANE %%%%%%%%%%%%%
    subplot(1, 3, 3)
    plot3(proximal_pc0(1, :), proximal_pc0(2, :), proximal_pc0(3, :), 'Color', [0.8500 0.3250 0.0980])
    hold on
%     trplot(eye(4), 'frame', 'Proximal', 'rgb', 'length', axis_scale*L);
    plot3(proximal_pc(1, :), proximal_pc(2, :), proximal_pc(3, :), 'Color', [0.8500 0.3250 0.0980])
%     trplot([eye(3), [0 0 L]'; zeros(1, 3), 1], 'frame', 'Distal', 'rgb', 'length', axis_scale*L);
    
    % Plot Actuators
    for i=1:na
        if(nargin > 3)
            % Longitudinal Cable
            if cableCat(i) == "l"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.0)
            % Oblique (or curved) Cable
            elseif cableCat(i) == "o"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2.0)
            % Oblique ClockWise
            elseif cableCat(i) == "ocw"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 2.0)
            % Oblique CounterClockWise
            elseif cableCat(i) == "occw"
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 2.0)
            %  Not Classified: Red
            else
                plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [1 0 0], 'LineWidth', 2.0)
            end
        else
            plot3(actuation_pc{i}(1, :), actuation_pc{i}(2, :), actuation_pc{i}(3, :), 'Color', [0 0 0], 'LineWidth', 2.0)
        end
    end
    % % Midline Plane
    s = surf(X, Y, Z); % Plot the surface
    s.EdgeColor = 'none';
    s.FaceAlpha = 0.3;
    s.FaceColor = [0.4660 0.6740 0.1880];
    
    text(geom_params.a0, geom_params.b0*0.9, L, "Dorsal");
    text(geom_params.a0, -geom_params.b0*0.9, L, "Ventral");
    
    % Body of Robot
    s_body = surf(X_body, Y_body, Z_body);
    s_body.EdgeColor = 'None';
    s_body.FaceAlpha = 0.5;
    s_body.FaceColor = [0.8500 0.3250 0.0980];
    
    grid on
    xlabel("x [m]")
    ylabel("y [m]")
    zlabel("z [m]")
    title("YZ Plane")
    axis equal
    view(90, 0) %yz
end

end