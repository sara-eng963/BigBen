%% Furuta Pendulum | Open-Loop Analytical Simulation (Week 6)
% PURPOSE
% Simulate the nonlinear four-state model produced by symbolic_lagrangian.m.
% This file contains NO equations of motion: teammates edit only the
% symbolic model, and this script provides numerical values and plots.
%
% Place this file next to symbolic_lagrangian.m in Modeling/.
% Requires Symbolic Math Toolbox for regenerating plantFcn.

%% 1. Generate the numerical plant from the symbolic model
clear; clc;
modelFolder = fileparts(mfilename('fullpath'));
run(fullfile(modelFolder, 'symbolic_lagrangian.m'));
% This creates plantFcn(x, Vm, params) in the workspace.
% The symbolic model also opens its equation viewer by default.

%% 2. Parameters: example test values or our hardware
% Use "test" to check the simulation workflow before hardware is ready.
% Switch to "hardware" once your teammates have measured the real values.
parameterMode = "test";   % "test" | "hardware"

switch parameterMode
    case "test"
        % Published numerical example: Cazzolato & Prime (2011),
        % "On the Dynamics of the Furuta Pendulum", Section 8, page 7.
        % DOI: https://doi.org/10.1155/2011/528341
        %
        % Paper notation: arm = link 1; pendulum = link 2.
        % J1 (paper) is the arm inertia about its COM; our I0 is about
        % the motor axis, so apply the parallel-axis theorem:
        % I0 = J1_paper + m_arm * l_arm_COM^2.
        % The paper's J2 maps to our pendulum COM inertia J1.
        %
        % IMPORTANT: these numerical values are only a starting example.
        % The paper uses a more complete 3D model than our tutorial model.
        % Running with these values does NOT validate our physics.
        p.I0 = 2.48e-2 + 0.300*(0.150)^2;  % [kg*m^2]
        p.J1 = 3.86e-3;                    % [kg*m^2]
        p.m1 = 0.075;                      % [kg]
        p.L0 = 0.278;                      % [m]
        p.l1 = 0.148;                      % [m]
        p.g  = 9.81;                       % [m/s^2]
        p.b0 = 1.00e-4;                    % [N*m*s/rad]
        p.b1 = 2.80e-4;                    % [N*m*s/rad]
        p.Rm = 7.80;                       % [ohm]
        p.Kt = 0.090;                      % [N*m/A]
        p.Ke = 0.090;                      % [V*s/rad], same SI constant

    case "hardware"
        % Replace NaN with your own documented measurements/estimates.
        p.I0 = NaN;      % Arm inertia about motor axis [kg*m^2]
        p.J1 = NaN;      % Pendulum transverse COM inertia [kg*m^2]
        p.m1 = NaN;      % Pendulum mass [kg]
        p.L0 = NaN;      % Motor axis to pendulum hinge [m]
        p.l1 = NaN;      % Pendulum hinge to center of mass [m]
        p.g  = 9.81;     % Gravity [m/s^2]
        p.b0 = NaN;      % Motor-axis viscous damping [N*m*s/rad]
        p.b1 = NaN;      % Pendulum hinge viscous damping [N*m*s/rad]
        p.Rm = NaN;      % DC motor winding resistance [ohm]
        p.Kt = NaN;      % DC motor torque constant [N*m/A]
        p.Ke = NaN;      % DC motor back-EMF constant [V*s/rad]

    otherwise
        error('parameterMode must be "test" or "hardware".');
end

% Exact input order required by plantFcn, from symbolic_lagrangian.m.
pNum = [p.I0; p.J1; p.m1; p.L0; p.l1; p.g; ...
        p.b0; p.b1; p.Rm; p.Kt; p.Ke];

if any(~isfinite(pNum))
    error('Missing numerical parameters for mode "%s". Fill in NaN values in Section 2.', parameterMode);
end
if p.Rm <= 0 || p.m1 <= 0 || p.L0 <= 0 || p.l1 <= 0 || ...
        p.I0 <= 0 || p.J1 <= 0 || p.Kt <= 0 || p.g <= 0 || ...
        p.b0 < 0 || p.b1 < 0
    error('Physical parameters must have valid positive/nonnegative values.');
end

fprintf('Open-loop parameter set: %s\n', parameterMode);

%% 3. Open-loop test settings
% State order: [arm angle; arm angular velocity; pendulum angle; pendulum angular velocity]
x0 = [0; 0; deg2rad(5); 0];  % 5-degree upright disturbance (example)
tspan = [0 5];               % Simulation duration [s]

% Applied voltage [V], with NO feedback control.
% NOTE: Vm=0 in this motor model still produces back-EMF braking torque.
voltageFcn = @(t) 1;

%% 4. Run nonlinear analytical simulation
rhs = @(t, state) plantFcn(state, voltageFcn(t), pNum);
options = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);
[t, X] = ode45(rhs, tspan, x0, options);

%% 5. Plot the open-loop response
figure('Name', 'Furuta | Open-Loop Analytical Response', ...
       'Color', 'w', 'NumberTitle', 'off');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact');

nexttile;
plot(t, rad2deg(X(:, [1 3])), 'LineWidth', 1.4);
ylabel('Angle [deg]');
legend('Rotary arm', 'Pendulum', 'Location', 'best');
grid on;
title('Joint Angles');

nexttile;
plot(t, rad2deg(X(:, [2 4])), 'LineWidth', 1.4);
xlabel('Time [s]');
ylabel('Angular velocity [deg/s]');
legend('Rotary arm', 'Pendulum', 'Location', 'best');
grid on;
title('Joint Angular Velocities');

sgtitle(tl, 'Open-Loop Nonlinear Analytical Model');

% t and X remain in the workspace for later comparison with Simscape.
