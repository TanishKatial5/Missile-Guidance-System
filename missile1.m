%% Missile Seeker with Sliding Mode Control (SMC)
% This code simulates a 2D engagement scenario with a missile using sliding mode control
% to intercept a target that moves at constant speed but variable heading angle.
% The simulation implements LOS (Line of Sight) guidance with SMC.

clear all; close all; clc;

%% Simulation Parameters
dt = 0.01;               % Time step [s]
t_final = 15;            % Simulation time [s]
t = 0:dt:t_final;        % Time vector
N = length(t);           % Number of simulation steps

%% Target Parameters
target.v = 300;          % Target speed [m/s]
target.x0 = 8000;        % Initial x-position [m]
target.y0 = 8000;        % Initial y-position [m]
target.heading0 = -135;  % Initial heading angle [deg]
target.period = 5;       % Period of heading changes [s]
target.heading_amplitude = 30; % Max change in heading [deg]
target.maneuver_start = 2; % Time when target starts maneuvering [s]

%% Missile Parameters
missile.v = 800;         % Missile speed [m/s]
missile.x0 = 0;          % Initial x-position [m]
missile.y0 = 0;          % Initial y-position [m]
missile.heading0 = 45;   % Initial heading angle [deg]
missile.anorm_max = 300; % Maximum normal acceleration [m/s^2]

%% SMC Parameters
smc.lambda = 5;          % Sliding surface slope
smc.eta = 200;           % Reaching law coefficient
smc.boundary_layer = 0.05; % Boundary layer thickness to reduce chattering

%% Initialize State Vectors
% Target state: [x, y, vx, vy]
target_state = zeros(N, 4);
target_state(1,:) = [target.x0, target.y0, ...
                     target.v*cosd(target.heading0), target.v*sind(target.heading0)];
target_heading = zeros(N, 1);
target_heading(1) = target.heading0;

% Missile state: [x, y, vx, vy]
missile_state = zeros(N, 4);
missile_state(1,:) = [missile.x0, missile.y0, ...
                      missile.v*cosd(missile.heading0), missile.v*sind(missile.heading0)];

% LOS parameters
los_angle = zeros(N, 1);
los_rate = zeros(N, 1);
range = zeros(N, 1);
sliding_surface = zeros(N, 1);

% Calculate initial LOS and range
dx = target_state(1,1) - missile_state(1,1);
dy = target_state(1,2) - missile_state(1,2);
range(1) = sqrt(dx^2 + dy^2);
los_angle(1) = atan2d(dy, dx);

%% Main Simulation Loop
for k = 1:N-1
    % Calculate current state
    dx = target_state(k,1) - missile_state(k,1);
    dy = target_state(k,2) - missile_state(k,2);
    range(k) = sqrt(dx^2 + dy^2);
    
    % Calculate LOS angle and rate
    los_angle(k) = atan2d(dy, dx);
    
    % Calculate relative velocity components
    vx_rel = target_state(k,3) - missile_state(k,3);
    vy_rel = target_state(k,4) - missile_state(k,4);
    
    % Calculate closing velocity
    v_closing = -(dx*vx_rel + dy*vy_rel)/range(k);
    
    % LOS rate calculation
    los_rate(k) = (dx*vy_rel - dy*vx_rel)/(range(k)^2);
    los_rate_rad = deg2rad(los_rate(k));
    
    % Define sliding surface (improved formulation)
    sliding_surface(k) = los_rate_rad + smc.lambda * sin(deg2rad(los_angle(k) - atan2d(missile_state(k,4), missile_state(k,3))));
    
    % Check for target interception
    if range(k) < 50
        fprintf('Target intercepted at time %.2f seconds\n', t(k));
        fprintf('Interception point: X = %.2f m, Y = %.2f m\n', missile_state(k,1), missile_state(k,2));
        fprintf('Final miss distance: %.2f m\n', range(k));
        break;
    end
    
    % Calculate SMC control input (normal acceleration command)
    % Add proportional term to improve convergence
    if abs(sliding_surface(k)) > smc.boundary_layer
        n_c = -sign(sliding_surface(k)) * smc.eta;
    else
        n_c = -sliding_surface(k) * (smc.eta/smc.boundary_layer);
    end
    
    % Add proportional-derivative term to improve performance
    los_rate_desired = 0; % We want LOS rate to be zero for collision course
    los_error = los_rate_rad - los_rate_desired;
    n_c = n_c - 50 * los_error;
    
    % Limit acceleration command to max value
    n_c = min(max(n_c, -missile.anorm_max), missile.anorm_max);
    
    % Calculate missile acceleration components
    % Normal acceleration is perpendicular to velocity vector
    v_mag = sqrt(missile_state(k,3)^2 + missile_state(k,4)^2);
    ax = -n_c * missile_state(k,4) / v_mag;
    ay = n_c * missile_state(k,3) / v_mag;
    
    % Update missile state
    missile_state(k+1,1) = missile_state(k,1) + missile_state(k,3) * dt;
    missile_state(k+1,2) = missile_state(k,2) + missile_state(k,4) * dt;
    missile_state(k+1,3) = missile_state(k,3) + ax * dt;
    missile_state(k+1,4) = missile_state(k,4) + ay * dt;
    
    % Maintain constant speed for missile
    v_current = sqrt(missile_state(k+1,3)^2 + missile_state(k+1,4)^2);
    missile_state(k+1,3) = missile_state(k+1,3) * missile.v / v_current;
    missile_state(k+1,4) = missile_state(k+1,4) * missile.v / v_current;
    
    % Update target heading angle (varying with time)
    % Only start maneuver after designated time
    if t(k+1) < target.maneuver_start
        target_heading(k+1) = target_heading(1);
    else
        target_heading(k+1) = target_heading(1) + ...
                          target.heading_amplitude * sin(2*pi*(t(k+1)-target.maneuver_start)/target.period);
    end
    
    % Update target state
    target_state(k+1,1) = target_state(k,1) + target_state(k,3) * dt;
    target_state(k+1,2) = target_state(k,2) + target_state(k,4) * dt;
    target_state(k+1,3) = target.v * cosd(target_heading(k+1));
    target_state(k+1,4) = target.v * sind(target_heading(k+1));
end

% Trim data to actual simulation length if interception occurred
if k < N-1
    final_idx = k;
    t = t(1:final_idx);
    range = range(1:final_idx);
    los_angle = los_angle(1:final_idx);
    los_rate = los_rate(1:final_idx);
    sliding_surface = sliding_surface(1:final_idx);
    missile_state = missile_state(1:final_idx, :);
    target_state = target_state(1:final_idx, :);
else
    final_idx = N;
end

%% Plotting Results
% Plot 1: Range vs Time
figure(1);
plot(t, range, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Range (m)');
title('Range between Missile and Target');

% Plot 2: LOS Angle vs Time
figure(2);
plot(t, los_angle, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('LOS Angle (deg)');
title('Line of Sight (LOS) Angle');

% Plot 3: Sliding Surface vs Time
figure(3);
plot(t, sliding_surface, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Sliding Surface Value');
title('Sliding Surface Evolution');
yline(smc.boundary_layer, '--r', 'Upper Boundary');
yline(-smc.boundary_layer, '--r', 'Lower Boundary');

% Plot 4: 2D Trajectories
figure(4);
plot(missile_state(:,1), missile_state(:,2), 'b-', 'LineWidth', 2);
hold on;
plot(target_state(:,1), target_state(:,2), 'r-', 'LineWidth', 2);
plot(missile_state(1,1), missile_state(1,2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot(target_state(1,1), target_state(1,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(missile_state(final_idx,1), missile_state(final_idx,2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
grid on;
xlabel('X-Position (m)');
ylabel('Y-Position (m)');
title('Missile and Target Trajectories');
legend('Missile Path', 'Target Path', 'Missile Start', 'Target Start', 'Interception Point');
axis equal;

% Plot 5: Missile and Target Headings
figure(5);
missile_heading = atan2d(missile_state(:,4), missile_state(:,3));
plot(t, missile_heading, 'b-', t, target_heading(1:length(t)), 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Heading Angle (deg)');
title('Missile and Target Heading Angles');
legend('Missile Heading', 'Target Heading');