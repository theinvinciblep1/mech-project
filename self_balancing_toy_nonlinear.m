    % MATLAB Simulation for Nonlinear Self-Balancing Toy

clc; clear; close all;

% Parameters
m = 0.5;  % Mass (kg)
g = 9.81; % Gravity (m/s^2)
l = 0.1;  % Length (m)
b = 0.05; % Damping coefficient

% Define ODE function (Fully Nonlinear Equation of Motion)
odefun = @(t, x) [x(2); - (b/m)*x(2) - (g/l)*sin(x(1))];

% Initial conditions (Large Angle Motion)
theta0 = 0.2; % Initial angle (radians) - Set to 1 rad or 2 rad to test large-angle motion
omega0 = 0; % Initial angular velocity
x0 = [theta0; omega0];

% Time span
tspan = linspace(0, 10, 500);

% Solve ODE using ode45
[t, x] = ode45(odefun, tspan, x0);

% Compute Energy Analysis
KE = 0.5 * m * (l * x(:,2)).^2;  % Kinetic Energy = 1/2 m v^2 (where v = l * theta_dot)
PE = m * g * l * (1 - cos(x(:,1))); % Potential Energy = mg(l - l*cos(theta))
TotalEnergy = KE + PE; % Total Mechanical Energy

% Phase Space Plot
figure;
plot(x(:,1), x(:,2), 'b', 'LineWidth', 2);
grid on;
xlabel('Angular Displacement (rad)');
ylabel('Angular Velocity (rad/s)');
title('Phase Space Plot');

% Energy Plot
figure;
plot(t, KE, 'r', 'LineWidth', 2); hold on;
plot(t, PE, 'b', 'LineWidth', 2);
plot(t, TotalEnergy, 'k--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Energy (J)');
legend('Kinetic Energy', 'Potential Energy', 'Total Energy');
title('Energy Analysis');

% Animation setup
figure;
axis equal;
hold on;
xlim([-l, l]); ylim([-l, l]);
grid on;
title('Live Animation: Nonlinear Self-Balancing Toy');
xlabel('X Position (m)'); ylabel('Y Position (m)');

% Toy pivot point
plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% Create toy rod
rod = line([0, l*sin(x(1,1))], [0, -l*cos(x(1,1))], 'LineWidth', 3, 'Color', 'b');

% Animation loop
for i = 1:length(t)
    % Update rod position
    rod.XData = [0, l*sin(x(i,1))];
    rod.YData = [0, -l*cos(x(i,1))];
    
    drawnow;
end
