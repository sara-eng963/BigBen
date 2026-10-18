%% Furuta Pendulum | Full Nonlinear Mechanical Model
% Self-contained MATLAB script using the full three-inertia equations.
%
% Coordinate convention
%   theta0 : arm azimuth, positive counterclockwise viewed from +z [rad]
%   theta1 : pendulum angle, theta1 = 0 upright and theta1 = pi downward [rad]
%   x      : [theta0; theta1; theta0dot; theta1dot]
%   tau    : effective mechanical torque at the arm axis [N*m]
%
% Model
%   M(q)*qdd + c(q,qd) + g(q) + d(qd) = [tau; 0]
%
% The numerical values below are synthetic validation values from the
% report. Replace them with measured/CAD values before using this model for
% hardware design or control tuning.

clearvars;
close all;
clc;

%% Parameters (SI units)
p.J0      = 0.0045;    % Effective arm/base inertia, excluding pendulum [kg*m^2]
p.m1      = 0.12;      % Pendulum mass [kg]
p.L0      = 0.18;      % Base axis to pendulum pivot [m]
p.lc      = 0.14;      % Pendulum pivot to center of mass [m]
p.J1r     = 0.0009;    % Pendulum COM inertia about hinge/radial axis [kg*m^2]
p.J1t     = 0.0009;    % Pendulum COM transverse inertia in swing plane [kg*m^2]
p.J1a     = 0.00003;   % Pendulum COM longitudinal inertia [kg*m^2]
p.b0      = 0.004;     % Arm viscous damping [N*m*s/rad]
p.b1      = 0.0015;    % Pendulum-joint viscous damping [N*m*s/rad]
p.gravity = 9.81;      % Gravitational acceleration [m/s^2]

validateParameters(p);

%% Simulation settings
tSpan = [0, 5];
x0 = [0; deg2rad(5); 0; 0];

% Mechanical arm-axis torque. Edit this function to apply another input.
tauFcn = @(t, x) 0.0;
% Example: a 0.02 N*m pulse from 0.5 s to 0.7 s:
% tauFcn = @(t, x) 0.02 .* (t >= 0.5 && t < 0.7);

solverOptions = odeset( ...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-11, ...
    'MaxStep', 0.01);

%% Simulate the nonlinear plant
[t, x] = ode45(@(t, x) furutaStateDerivative(t, x, tauFcn, p), ...
    tSpan, x0, solverOptions);

sampleCount = numel(t);
tau = zeros(sampleCount, 1);
energy = zeros(sampleCount, 1);
massMatrixDeterminant = zeros(sampleCount, 1);

for k = 1:sampleCount
    tau(k) = tauFcn(t(k), x(k, :).');
    [energy(k), massMatrixDeterminant(k)] = ...
        furutaEnergyAndDeterminant(x(k, :).', p);
end

assert(all(massMatrixDeterminant > 0), ...
    'The mass matrix became singular or indefinite during simulation.');

% Results are kept in one structure for convenient saving or post-processing.
furutaResult = struct( ...
    'time', t, ...
    'state', x, ...
    'torque', tau, ...
    'energy', energy, ...
    'massMatrixDeterminant', massMatrixDeterminant, ...
    'parameters', p, ...
    'stateOrder', {{'theta0', 'theta1', 'theta0dot', 'theta1dot'}});

%% Plot results
figure('Color', 'w', 'Name', 'Nonlinear Furuta Pendulum');

subplot(2, 2, 1);
plot(t, rad2deg(x(:, 1)), 'LineWidth', 1.4);
hold on;
plot(t, rad2deg(x(:, 2)), 'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('\theta_0 arm', '\theta_1 pendulum', 'Location', 'best');
title('Generalized coordinates');

subplot(2, 2, 2);
plot(t, x(:, 3), 'LineWidth', 1.4);
hold on;
plot(t, x(:, 4), 'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
legend('d\theta_0/dt', 'd\theta_1/dt', 'Location', 'best');
title('Generalized velocities');

subplot(2, 2, 3);
plot(t, tau, 'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Torque [N*m]');
title('Mechanical input torque');

subplot(2, 2, 4);
plot(t, energy, 'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Mechanical energy [J]');
title('Total energy');

%% Local model functions
function xdot = furutaStateDerivative(t, x, tauFcn, p)
%FURUTASTATEDERIVATIVE Evaluate the report's nonlinear state equation.

theta1 = x(2);
theta0dot = x(3);
theta1dot = x(4);

s1 = sin(theta1);
c1 = cos(theta1);

D = p.J1r + p.m1*p.lc^2;
B = p.m1*p.L0*p.lc;
delta = p.m1*p.lc^2 + p.J1t - p.J1a;
A = p.J0 + p.m1*p.L0^2 ...
    + (p.m1*p.lc^2 + p.J1t)*s1^2 + p.J1a*c1^2;

M = [A, B*c1; ...
     B*c1, D];

c = [2*delta*s1*c1*theta0dot*theta1dot - B*s1*theta1dot^2; ...
    -delta*s1*c1*theta0dot^2];

g = [0; ...
    -p.m1*p.gravity*p.lc*s1];

d = [p.b0*theta0dot; ...
     p.b1*theta1dot];

tau = tauFcn(t, x);
qdd = M \ ([tau; 0] - c - g - d);
xdot = [theta0dot; theta1dot; qdd];
end

function [energy, detM] = furutaEnergyAndDeterminant(x, p)
%FURUTAENERGYANDDETERMINANT Compute total energy and det(M).

theta1 = x(2);
theta0dot = x(3);
theta1dot = x(4);

s1 = sin(theta1);
c1 = cos(theta1);

D = p.J1r + p.m1*p.lc^2;
B = p.m1*p.L0*p.lc;
A = p.J0 + p.m1*p.L0^2 ...
    + (p.m1*p.lc^2 + p.J1t)*s1^2 + p.J1a*c1^2;

kineticEnergy = 0.5*A*theta0dot^2 ...
    + B*c1*theta0dot*theta1dot ...
    + 0.5*D*theta1dot^2;
potentialEnergy = p.m1*p.gravity*p.lc*(1 + c1);

energy = kineticEnergy + potentialEnergy;
detM = A*D - (B*c1)^2;
end

function validateParameters(p)
%VALIDATEPARAMETERS Reject nonphysical values before integration.

requiredFields = {'J0', 'm1', 'L0', 'lc', 'J1r', 'J1t', 'J1a', ...
    'b0', 'b1', 'gravity'};

assert(all(isfield(p, requiredFields)), ...
    'The parameter structure is missing one or more required fields.');

strictlyPositive = [p.J0, p.m1, p.L0, p.lc, p.J1r, p.J1t, p.gravity];
nonnegative = [p.J1a, p.b0, p.b1];

assert(all(isfinite(strictlyPositive)) && all(strictlyPositive > 0), ...
    'Mass, lengths, gravity, J0, J1r, and J1t must be finite and positive.');
assert(all(isfinite(nonnegative)) && all(nonnegative >= 0), ...
    'J1a and viscous damping coefficients must be finite and nonnegative.');

testAngles = linspace(-pi, pi, 721);
for angleIndex = 1:numel(testAngles)
    theta1 = testAngles(angleIndex);
    A = p.J0 + p.m1*p.L0^2 ...
        + (p.m1*p.lc^2 + p.J1t)*sin(theta1)^2 ...
        + p.J1a*cos(theta1)^2;
    D = p.J1r + p.m1*p.lc^2;
    B = p.m1*p.L0*p.lc;
    assert(A*D - (B*cos(theta1))^2 > 0, ...
        'The mass matrix is not positive definite for all pendulum angles.');
end
end
