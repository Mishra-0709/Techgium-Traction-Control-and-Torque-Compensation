% MATLAB Code to Obtain Power Loss and Slip Characteristics with Traction Control

% Vehicle Parameters
mass = 1500;            % Vehicle mass (kg)
g = 9.81;               % Acceleration due to gravity (m/s^2)
wheel_radius = 0.3;     % Wheel radius (m)
moment_of_inertia = 1.2; % Moment of inertia of the wheel (kg.m^2)
engine_torque = 300;    % Maximum engine torque (Nm)
drag_coeff = 0.3;       % Aerodynamic drag coefficient
air_density = 1.225;    % Air density (kg/m^3)
frontal_area = 2.2;     % Frontal area (m^2)
rolling_resistance_coeff = 0.015;
slip_target = 0.1;      % Desired slip ratio for traction control
control_gain = 500;     % Traction control proportional gain

% Simulation Parameters
time_step = 0.01;       % Time step (s)
total_time = 10;        % Total simulation time (s)
num_steps = total_time / time_step;
time = linspace(0, total_time, num_steps);

% Initial Conditions
vehicle_speed = 0;      % Initial vehicle speed (m/s)
angular_velocity = 0;   % Initial wheel angular velocity (rad/s)

% Arrays to Store Results
slip_ratio = zeros(1, num_steps);
torque_at_wheel = zeros(1, num_steps);
power_loss = zeros(1, num_steps);
vehicle_speeds = zeros(1, num_steps);

for i = 1:num_steps
    % Aerodynamic Drag and Rolling Resistance
    drag_force = 0.5 * drag_coeff * air_density * frontal_area * vehicle_speed^2;
    rolling_resistance = rolling_resistance_coeff * mass * g;
    
    % Net Force on the Vehicle
    traction_force = (engine_torque / wheel_radius) - drag_force - rolling_resistance;
    
    % Vehicle Acceleration (F = ma)
    acceleration = traction_force / mass;
    
    % Update Vehicle Speed
    vehicle_speed = vehicle_speed + acceleration * time_step;
    vehicle_speeds(i) = vehicle_speed;
    
    % Update Wheel Angular Velocity
    wheel_linear_speed = angular_velocity * wheel_radius;
    
    % Compute Slip Ratio
    slip_ratio_current = (wheel_linear_speed - vehicle_speed) / max(vehicle_speed, 1e-3); % Avoid division by zero
    slip_ratio(i) = slip_ratio_current;
    
    % Apply Traction Control to Adjust Engine Torque
    torque_correction = control_gain * (slip_ratio_current - slip_target);
    engine_torque = max(0, engine_torque - torque_correction); % Limit torque to non-negative values
    
    % Update Angular Velocity
    angular_velocity = angular_velocity + (engine_torque - moment_of_inertia * angular_velocity) / moment_of_inertia * time_step;
    
    % Torque at the Wheel and Power Loss
    torque_at_wheel(i) = engine_torque;
    power_loss(i) = torque_correction * angular_velocity; % Power loss due to traction control adjustment
end

% Plot Results
figure;
subplot(3,1,1);
plot(time, slip_ratio);
title('Slip Ratio vs Time');
xlabel('Time (s)');
ylabel('Slip Ratio');
grid on;

subplot(3,1,2);
plot(time, torque_at_wheel);
title('Torque at Wheel vs Time');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

subplot(3,1,3);
plot(time, power_loss);
title('Power Loss vs Time');
xlabel('Time (s)');
ylabel('Power Loss (W)');
grid on;
