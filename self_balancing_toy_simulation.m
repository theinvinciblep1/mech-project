% MATLAB Simulation for Self-Balancing Toy

clc; clear; close all;

% Parameters (adjustable)
m = 0.5;  % Mass (kg)
g = 9.81; % Gravity (m/s^2)
l = 0.1;  % Length (m)
b = 0.05; % Damping coefficient (adjust as needed)

% Define the equation of motion: theta'' + (b/m)*theta' + (g/l)*sin(theta) = 0
% Convert to first-order system: x1 = theta, x2 = theta'

% Define ODE function
odefun = @(t, x) [x(2); - (b/m)*x(2) - (g/l)*sin(x(1))];

% Initial conditions: Small displacement & zero initial velocity
theta0 = 0.2; % Initial angle (radians)
omega0 = 0;   % Initial angular velocity
x0 = [theta0; omega0];

% Time span for simulation
tspan = [0 10];

% Solve ODE using ode45
[t, x] = ode45(odefun, tspan, x0);

% Plot results: Small Oscillations
figure;
plot(t, x(:,1), 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Theta (rad)');
title('Small Oscillations');
grid on;

% Damped Oscillations (adjust damping and re-simulate)
b_values = [0.01, 0.1, 1]; % Underdamped, Critically Damped, Overdamped

figure;
hold on;
for i = 1:length(b_values)
    b = b_values(i);
    odefun_damped = @(t, x) [x(2); - (b/m)*x(2) - (g/l)*sin(x(1))];
    [t, x] = ode45(odefun_damped, tspan, x0);
    plot(t, x(:,1), 'LineWidth', 1.5);
end
xlabel('Time (s)'); ylabel('Theta (rad)');
title('Damped Oscillations');
legend('Underdamped', 'Critically Damped', 'Overdamped');
grid on;

% Phase Space Plot
figure;
plot(x(:,1), x(:,2), 'r', 'LineWidth', 1.5);
xlabel('Theta (rad)'); ylabel('Angular Velocity (rad/s)');
title('Phase Space Plot');
grid on;

% Surface Plot for Stability Visualization
[Theta, Omega] = meshgrid(linspace(-pi, pi, 50), linspace(-5, 5, 50));
Potential = m * g * l * (1 - cos(Theta));
figure;
surf(Theta, Omega, Potential, 'EdgeColor', 'none');
xlabel('Theta (rad)'); ylabel('Angular Velocity (rad/s)'); zlabel('Potential Energy');
title('Surface Plot of Stability');
colorbar;
grid on;
