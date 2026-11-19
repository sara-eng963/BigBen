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


%% 2. Parameters
% Reference:
% Cazzolato & Prime (2011), "On the Dynamics of the
% Furuta Pendulum", Section 8.
% DOI: 10.1155/2011/528341
%
% Literature parameters are for model testing.
% They are NOT measurements of our physical prototype.

parameterMode = "test";  % "test" | "hardware"

switch parameterMode

    case "test"

        % --- Mechanical parameters ---
        % Paper: Arm 1 = horizontal arm
        %        Arm 2 = pendulum

        % Horizontal arm inertia about motor axis
        % Parallel-axis theorem: J0 = J1 + m_arm*l_arm^2
        p.J0 = 2.48e-2 + 0.300*(0.150)^2;  % [kg*m^2]

        % Pendulum principal moments of inertia
        % Paper: J2yy = J2zz = J2
        % Longitudinal inertia neglected (slender-link assumption)
        p.J1r = 3.86e-3;     % Hinge-axis inertia [kg*m^2]
        p.J1t = 3.86e-3;     % Transverse inertia [kg*m^2]
        p.J1a = 0;          % Longitudinal inertia [kg*m^2]

        p.m1 = 0.075;       % Pendulum mass [kg]
        p.L0 = 0.278;       % Motor axis to pendulum hinge [m]
        p.l1 = 0.148;       % Pendulum hinge to COM [m]
        p.g  = 9.81;        % Gravity [m/s^2]

        % --- Viscous friction ---
        p.b0 = 1.00e-4;     % Arm damping [N*m*s/rad]
        p.b1 = 2.80e-4;     % Pendulum damping [N*m*s/rad]

        % --- DC motor parameters ---
        p.Rm = 7.80;       % Motor resistance [ohm]
        p.Kt = 0.090;      % Torque constant [N*m/A]
        p.Ke = 0.090;      % Back-EMF constant [V*s/rad]

        % Paper also specifies motor inductance Lm = 0.005 H.
        % Our current 4-state model NEGLECTS inductance.

    case "hardware"

        % Replace with actual measured/CAD-derived parameters.
        p.J0  = NaN;
        p.J1r = NaN;
        p.J1t = NaN;
        p.J1a = NaN;
        p.m1  = NaN;
        p.L0  = NaN;
        p.l1  = NaN;
        p.g   = 9.81;

        p.b0 = NaN;
        p.b1 = NaN;

        p.Rm = NaN;
        p.Kt = NaN;
        p.Ke = NaN;

    otherwise
        error('Invalid parameter mode.');
end

%% Parameter vector
% MUST match the exact order in symbolic_lagrangian.m

pNum = [p.J0; p.J1r; p.J1t; p.J1a; ...
        p.m1; p.L0; p.l1; p.g; ...
        p.b0; p.b1; p.Rm; p.Kt; p.Ke];

%% Parameter validation
if any(~isfinite(pNum))
    error('Missing or non-finite physical parameters.');
end

positiveParams = [p.J0, p.J1r, p.J1t, p.m1, ...
                  p.L0, p.l1, p.g, p.Rm, p.Kt, p.Ke];

nonnegativeParams = [p.J1a, p.b0, p.b1];

if any(positiveParams <= 0) || any(nonnegativeParams < 0)
    error('Nonphysical parameter values.');
end

fprintf('Parameter source: %s\n', parameterMode);

%% 3. Open-loop test settings
% State order: [arm angle; arm angular velocity; pendulum angle; pendulum angular velocity]
x0 = [0; 0; pi + deg2rad(5); 0];  % 5-degree upright disturbance (example)
tspan = [0 20];               % Simulation duration [s]

% Applied voltage [V], with NO feedback control.
% NOTE: Vm=0 in this motor model still produces back-EMF braking torque.
voltageFcn = @(t) 0;  % Zero applied motor voltage

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
% White background, black axes, gray grid
set(findall(gcf, 'Type', 'axes'), ...
    'Color', 'w', ...
    'XColor', 'k', ...
    'YColor', 'k', ...
    'GridColor', [0.75 0.75 0.75]);

% White legend with black text
set(findall(gcf, 'Type', 'legend'), ...
    'Color', 'w', ...
    'TextColor', 'k', ...
    'EdgeColor', [0.5 0.5 0.5]);

% Black titles and labels
set(findall(gcf, 'Type', 'text'), 'Color', 'k');

% t and X remain in the workspace for later comparison with Simscape.


%% 6. Open-Loop Dynamic Analysis
% Analyze small oscillations around downward equilibrium.
% theta1 = pi, Vm = 0.

fprintf('\n===== OPEN-LOOP DYNAMIC ANALYSIS =====\n');

%% A. Linearized model eigenvalues
% All equations originate from symbolic_lagrangian.m.

x_eq = [0; 0; pi; 0];
u_eq = 0;

A_sym = jacobian(xdot, x);

A_down = double(subs(A_sym, ...
    [x; u; params], [x_eq; u_eq; pNum]));

poles = eig(A_down);

fprintf('\nOpen-loop eigenvalues:\n');
disp(poles);

% Identify the oscillatory mode (positive imaginary pole)
oscPoles = poles(imag(poles) > 1e-8);

if isempty(oscPoles)
    error('No oscillatory mode detected.');
end

lambda = oscPoles(1);

sigma = real(lambda);
omega_d = abs(imag(lambda));
omega_n = abs(lambda);
zeta = -sigma / omega_n;

frequency = omega_d / (2*pi);
period = 2*pi / omega_d;

fprintf('\n--- Linearized Model ---\n');
fprintf('Natural frequency wn  = %.4f rad/s\n', omega_n);
fprintf('Damped frequency wd   = %.4f rad/s\n', omega_d);
fprintf('Frequency             = %.4f Hz\n', frequency);
fprintf('Period                = %.4f s\n', period);
fprintf('Damping ratio zeta    = %.6f\n', zeta);

if sigma < 0
    fprintf('Amplitude half-life   = %.3f s\n', log(2)/(-sigma));
    fprintf('Decay per cycle       = %.2f %%\n', ...
        100*(1-exp(sigma*period)));
end

%% B. Measure damping from nonlinear simulation
% Pendulum angular deviation from downward equilibrium

errorAngle = X(:,3) - pi;

% Find positive oscillation peaks without extra toolboxes
peakIdx = find( ...
    errorAngle(2:end-1) > errorAngle(1:end-2) & ...
    errorAngle(2:end-1) >= errorAngle(3:end) & ...
    errorAngle(2:end-1) > 0) + 1;

peakTimes = t(peakIdx);
peakAmps = errorAngle(peakIdx);

if numel(peakIdx) >= 3

    % Period and frequency from successive positive peaks
    T_measured = mean(diff(peakTimes));
    wd_measured = 2*pi/T_measured;

    % Fit exponential decay: amplitude = A0*exp(-alpha*t)
    fitCoeffs = polyfit(peakTimes, log(peakAmps), 1);
    alpha = -fitCoeffs(1);

    % Damping ratio and natural frequency
    wn_measured = hypot(wd_measured, alpha);
    zeta_measured = alpha/wn_measured;

    decayPercent = 100*(1-exp(-alpha*T_measured));

    fprintf('\n--- Nonlinear Simulation Measurements ---\n');
    fprintf('Detected peaks        = %d\n', numel(peakIdx));
    fprintf('Measured period       = %.4f s\n', T_measured);
    fprintf('Measured frequency    = %.4f Hz\n', 1/T_measured);
    fprintf('Measured wn           = %.4f rad/s\n', wn_measured);
    fprintf('Measured zeta         = %.6f\n', zeta_measured);
    fprintf('Amplitude decay/cycle = %.2f %%\n', decayPercent);
    fprintf('First peak            = %.4f deg\n', rad2deg(peakAmps(1)));
    fprintf('Last peak             = %.4f deg\n', rad2deg(peakAmps(end)));

else
    warning('Not enough oscillation peaks for damping measurement.');
end


%% 7. S-Plane Pole-Zero Map
% Linearized model around downward equilibrium

% Motor voltage input matrix
B_sym = jacobian(xdot, u);

B_down = double(subs(B_sym, ...
    [x; u; params], [x_eq; u_eq; pNum]));

% Measured outputs: arm and pendulum angles
C_down = [1 0 0 0;
          0 0 1 0];

D_down = zeros(2,1);

% Create continuous-time state-space model
sys_down = ss(A_down, B_down, C_down, D_down);

fig = figure('Name', 'Furuta | Open-Loop S-Plane');
theme(fig, "light");

pzp = pzplot(sys_down);
pzp.AxesStyle.GridVisible = "on";

title('Open-Loop Pole-Zero Map - Downward Equilibrium');
