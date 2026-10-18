%% Furuta Pendulum - Symbolic Lagrangian Model
% Simplified mechanical model from MCT411 Section 2, Sheet 2, Question 4.
% Includes viscous joint friction and a directly coupled brushed DC motor.
% All equations in the viewer are generated from symbolic expressions.
% Re-run this script after changing the model to refresh the viewer.
%
% Assumptions:
% - theta1 = 0 is the upright position.
% - Tutorial's simplified planar COM-velocity model (not full 3-D dynamics).
% - Rigid arm, viscous joint friction, no pendulum motor.
% - Direct-drive DC motor; gearbox and rotor inertia are not modeled.
% - Motor inductance Lm is retained, so motor current is a fifth state.

%% 1. Symbolic Variables
clear; clc;

% Generalized coordinates, velocities, and accelerations
syms theta0 theta1 real
syms dtheta0 dtheta1 real
syms ddtheta0 ddtheta1 real

% Mechanical parameters and motor torque
syms I0 J1 m1 L0 l1 g tau real

% Viscous friction coefficients
syms b0 b1 real

% DC motor parameters
syms Rm Lm Kt Ke real

% Motor voltage, current, and rate of change of current
syms Vm im dim real

q   = [theta0; theta1];
dq  = [dtheta0; dtheta1];
ddq = [ddtheta0; ddtheta1];

%% 2. Energy Equations

% Arm rotation about the motor axis
T_arm = sym(1)/2 * I0 * dtheta0^2;

% Pendulum rotation about its center of mass (tutorial approximation)
T_pend_rot = sym(1)/2 * J1 * dtheta1^2;

% Pendulum COM velocity (tutorial's simplified planar expression)
vx = L0*dtheta0 + l1*dtheta1*cos(theta1);
vy = l1*dtheta1*sin(theta1);

% Pendulum COM translational kinetic energy
T_pend_trans = sym(1)/2 * m1 * (vx^2 + vy^2);

T = simplify(T_arm + T_pend_rot + T_pend_trans);
V = m1*g*l1*cos(theta1);
Lagr = simplify(T - V);

%% 3. Euler-Lagrange Mechanical Equations
% Friction is a nonconservative generalized force; it is not part of T or V.
Q = [tau - b0*dtheta0;
     -b1*dtheta1];

% Conservative Euler-Lagrange expressions, before generalized forces
EOM = sym(zeros(2, 1));

for i = 1:numel(q)
    dL_ddq = diff(Lagr, dq(i));

    % Total time derivative, using the multivariable chain rule
    dt_term = jacobian(dL_ddq, q)*dq ...
            + jacobian(dL_ddq, dq)*ddq;

    dL_dq = diff(Lagr, q(i));
    EOM(i) = simplify(dt_term - dL_dq);
end

% These equations have motor torque tau as their input
mechanical_EOM = EOM == Q;

%% 4. DC Motor and Coupled Mechanical Equations
% Direct-drive DC motor: Vm = Lm*dim + Rm*im + Ke*dtheta0.
motor_EOM = Lm*dim + Rm*im + Ke*dtheta0 == Vm;
motor_torque = tau == Kt*im;

% Current differential equation and motor torque substitution
dim_solution = simplify(solve(motor_EOM, dim));
current_EOM = dim == dim_solution;

% Coupled mechanical equations and generalized forces (in motor current)
coupled_EOM = subs(mechanical_EOM, tau, Kt*im);
Q_coupled = subs(Q, tau, Kt*im);

% State and input definitions; state derivative is derived in a later stage
x = [q; dq; im];
u = Vm;

%% 5. Symbolically Generated Equation Viewer
% No mathematical equations are hard-coded as LaTeX strings below.
% MATLAB's symbolic latex() output is formatted only for display.

% Close the previous viewer before rebuilding it on a subsequent run
oldFig = findall(groot, 'Type', 'figure', ...
    'Tag', 'FurutaSymbolicEquationViewer');
if ~isempty(oldFig)
    delete(oldFig);
end

fig = figure( ...
    'Name', 'Furuta Pendulum | Symbolic Mathematical Model', ...
    'Tag', 'FurutaSymbolicEquationViewer', ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [150 150 1120 570]);

tabs = uitabgroup(fig, ...
    'Units', 'normalized', ...
    'Position', [0.015 0.025 0.97 0.95]);

% Explicit tab backgrounds are essential when MATLAB uses a dark theme.
% The figure Color property alone does not control the tab panels.
energyTab = uitab(tabs, 'Title', 'Energy & Lagrangian', ...
    'BackgroundColor', [1 1 1]);
mechanicalTab = uitab(tabs, 'Title', 'Mechanical Dynamics', ...
    'BackgroundColor', [1 1 1]);
motorTab = uitab(tabs, 'Title', 'Motor & States', ...
    'BackgroundColor', [1 1 1]);

% ------------------ Tab 1: Energy ------------------
ax1 = equationAxes(energyTab);
drawHeading(ax1, 0.91, 'Energy & Lagrangian');
drawHeading(ax1, 0.79, 'Total kinetic energy', 11);
drawEquation(ax1, 0.69, ['T=' symbolicLatex(T)], 14);
drawHeading(ax1, 0.51, 'Gravitational potential energy', 11);
drawEquation(ax1, 0.42, ['V=' symbolicLatex(V)], 16);
drawHeading(ax1, 0.27, 'Lagrangian', 11);
drawEquation(ax1, 0.17, ['\mathcal{L}=' symbolicLatex(Lagr)], 13);

% ---------------- Tab 2: Mechanical ----------------
ax2 = equationAxes(mechanicalTab);
drawHeading(ax2, 0.91, 'Nonlinear Mechanical Equations');
drawHeading(ax2, 0.77, 'Rotary arm', 11);
drawEquation(ax2, 0.68, symbolicLatex(coupled_EOM(1)), 14);
drawHeading(ax2, 0.53, 'Pendulum', 11);
drawEquation(ax2, 0.44, symbolicLatex(coupled_EOM(2)), 14);
drawHeading(ax2, 0.28, 'Generalized forces (with friction)', 11);
drawEquation(ax2, 0.19, ['Q_0=' symbolicLatex(Q_coupled(1))], 15);
drawEquation(ax2, 0.09, ['Q_1=' symbolicLatex(Q_coupled(2))], 15);

% ---------------- Tab 3: Motor & States ------------
ax3 = equationAxes(motorTab);
drawHeading(ax3, 0.91, 'DC Motor & State Definitions');
drawHeading(ax3, 0.78, 'Electrical dynamics', 11);
drawEquation(ax3, 0.70, symbolicLatex(motor_EOM), 15);
drawHeading(ax3, 0.58, 'Electromagnetic torque', 11);
drawEquation(ax3, 0.50, symbolicLatex(motor_torque), 15);
drawHeading(ax3, 0.39, 'Current differential equation', 11);
drawEquation(ax3, 0.31, symbolicLatex(current_EOM), 15);
drawHeading(ax3, 0.20, 'State vector and control input', 11);
drawEquation(ax3, 0.12, stateVectorLatex(x), 14);
drawEquation(ax3, 0.045, ['u=' symbolicLatex(u)], 14);

%% Local Display Helpers
% In MATLAB scripts, local functions must appear after all script sections.

function ax = equationAxes(parentTab)
    ax = axes('Parent', parentTab, ...
        'Position', [0.04 0.05 0.92 0.90], ...
        'Color', 'w', ...
        'Visible', 'off');
    axis(ax, [0 1 0 1]);
end

function drawHeading(ax, y, titleText, fontSize)
    if nargin < 4
        fontSize = 19;
    end
    text(ax, 0.02, y, titleText, ...
        'Units', 'normalized', ...
        'Interpreter', 'none', ...
        'FontSize', fontSize, ...
        'FontWeight', 'bold', ...
        'Color', 'k', ...
        'VerticalAlignment', 'middle');
end

function drawEquation(ax, y, latexBody, preferredSize)
    % A conservative size reduction for especially long equations.
    fontSize = preferredSize;
    if numel(latexBody) > 360
        fontSize = min(fontSize, 10);
    elseif numel(latexBody) > 260
        fontSize = min(fontSize, 11);
    elseif numel(latexBody) > 190
        fontSize = min(fontSize, 12);
    end
    text(ax, 0.02, y, ['$$' latexBody '$$'], ...
        'Units', 'normalized', ...
        'Interpreter', 'latex', ...
        'FontSize', fontSize, ...
        'Color', 'k', ...
        'VerticalAlignment', 'middle');
end

function out = stateVectorLatex(state)
    % A horizontal list avoids unsupported \begin{bmatrix} in figure text.
    elements = cell(1, numel(state));
    for k = 1:numel(state)
        elements{k} = symbolicLatex(state(k));
    end
    out = ['x=\left[' strjoin(elements, ',\;') '\right]^{T}'];
end

function out = symbolicLatex(expression)
    % Format MATLAB's symbolic output, without modifying the equations.
    out = latex(expression);

    names = { ...
        'ddtheta0','ddtheta1','dtheta0','dtheta1', ...
        'theta0','theta1','dim','im','Vm','Rm','Lm', ...
        'Kt','Ke','b0','b1','I0','J1','L0','l1','m1','tau'};

    notation = { ...
        '\ddot{\theta}_{0}','\ddot{\theta}_{1}', ...
        '\dot{\theta}_{0}','\dot{\theta}_{1}', ...
        '\theta_{0}','\theta_{1}', ...
        '\dot{i}_{m}','i_{m}','V_{m}','R_{m}','L_{m}', ...
        'K_{t}','K_{e}','b_{0}','b_{1}', ...
        'I_{0}','J_{1}','L_{0}','l_{1}','m_{1}','\tau'};

    % Convert source symbol names to unique placeholders first.
    % This prevents replacements inside previously formatted LaTeX.
    markers = cell(size(names));
    for k = 1:numel(names)
        name = names{k};
        markers{k} = sprintf('ZZSYMBOL%02dZZ', k);
        marker = markers{k};

        % Typical MATLAB latex() output: \mathrm{ddtheta}_{0}.
        if isstrprop(name(end), 'digit')
            base = name(1:end-1);
            digit = name(end);
            out = strrep(out, ['\mathrm{' base '}_{' digit '}'], marker);
            out = strrep(out, ['\mathit{' base '}_{' digit '}'], marker);
            % Leave native Greek commands (such as \theta_{0}) intact.
            if ~strcmp(base, 'theta')
                out = strrep(out, [base '_{' digit '}'], marker);
            end
        end

        % Alternative MATLAB symbolic variable representations.
        out = strrep(out, ['\mathrm{' name '}'], marker);
        out = strrep(out, ['\mathit{' name '}'], marker);
        out = strrep(out, ['\operatorname{' name '}'], marker);

        % Literal symbol identifiers, but not fragments of LaTeX commands.
        pattern = ['(?<![A-Za-z\\])' regexptranslate('escape', name) ...
                   '(?![A-Za-z0-9])'];
        out = regexprep(out, pattern, marker);
    end

    % Finally replace placeholders with formatted mathematical symbols.
    for k = 1:numel(names)
        out = strrep(out, markers{k}, notation{k});
    end
end
