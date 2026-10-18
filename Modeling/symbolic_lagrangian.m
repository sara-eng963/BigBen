%% Furuta Pendulum | Symbolic Model Template (Week 6)
% PURPOSE
% Teammates edit the mathematical assumptions and equations in Sections 2-3.
% Sections 4-5 derive a nonlinear 4-state plant and display the results.
% No numerical hardware values, controllers, or simulation tests are defined here.
%
% INITIAL MODEL (replaceable): MCT411 Sheet 2, Q4 simplified energy model.
% theta1 = 0 is upright. The tutorial uses simplified planar COM velocities;
% this is NOT the full three-dimensional Furuta pendulum dynamics.
% The DC motor is assumed direct-drive and its inductance is neglected.
%
% OUTPUTS IN MATLAB WORKSPACE
%   mechanicalEqs   : 2 Euler-Lagrange equations (with friction)
%   voltageEqs      : 2 mechanical equations with voltage as input
%   M, h            : mass matrix and other mechanical dynamics terms
%   x, xdot         : 4-state vector and nonlinear state derivatives
%   params          : parameter vector (ordered for numerical use)
%   plantFcn(x, Vm, params) : numerical nonlinear derivative function
%
% Requires Symbolic Math Toolbox. Re-run to regenerate all expressions.

%% 1. Symbols (simulation interface -- keep state order consistent)
syms theta0 theta1 dtheta0 dtheta1 ddtheta0 ddtheta1 real
syms I0 J1 m1 L0 l1 g b0 b1 real
syms tau Vm im Rm Kt Ke real

q   = [theta0; theta1];
dq  = [dtheta0; dtheta1];
ddq = [ddtheta0; ddtheta1];

% Fixed state order: [arm angle; arm speed; pendulum angle; pendulum speed].
x = [theta0; dtheta0; theta1; dtheta1];
u = Vm;  % voltage input (not a state)

%% 2. Energy model -- TEAMMATES EDIT HERE
% Replace T and V when the agreed mechanical derivation changes.
% Example only: energy from the course tutorial's simplified model.

T_arm = sym(1)/2 * I0 * dtheta0^2;
T_pend_rot = sym(1)/2 * J1 * dtheta1^2;

% Tutorial's APPROXIMATE planar center-of-mass velocity.
vx = L0*dtheta0 + l1*dtheta1*cos(theta1);
vy = l1*dtheta1*sin(theta1);
T_pend_trans = sym(1)/2 * m1 * (vx^2 + vy^2);

T = simplify(T_arm + T_pend_rot + T_pend_trans);
V = m1*g*l1*cos(theta1);

%% 3. Joint forces and motor model -- TEAMMATES EDIT HERE
% Friction: viscous torque opposes motion at both joints.
% Passive pendulum has no applied motor torque.
Q = [tau - b0*dtheta0;
     -b1*dtheta1];

% Simplified DC motor: Lm = 0, direct-drive, tau = Kt * im.
% Because current is algebraic, the plant has FOUR states, not five.
motorVoltageEq = Vm == Rm*im + Ke*dtheta0;
motorTorqueEq  = tau == Kt*im;

%% 4. Automatic symbolic derivation -- DO NOT EDIT
Lagr = simplify(T - V);
EOM = sym(zeros(2,1));
for k = 1:numel(q)
    dL_ddq = diff(Lagr, dq(k));
    timeDerivative = jacobian(dL_ddq, q)*dq ...
                   + jacobian(dL_ddq, dq)*ddq;
    EOM(k) = simplify(timeDerivative - diff(Lagr, q(k)));
end

mechanicalEqs = EOM == Q;

% Remove motor current algebraically and use voltage as plant input.
currentExpr = simplify(solve(motorVoltageEq, im));
torqueExpr  = simplify(subs(rhs(motorTorqueEq), im, currentExpr));
voltageEqs = EOM == subs(Q, tau, torqueExpr);

% Extract M(q)*ddq + h(q,dq) = generalized forces.
M = simplify(jacobian(EOM, ddq));
h = simplify(EOM - M*ddq);
Qvoltage = simplify(subs(Q, tau, torqueExpr));
accelerations = simplify(M \ (Qvoltage - h));

% Nonlinear four-state model: xdot = f(x, Vm, params).
xdot = simplify([dtheta0; accelerations(1); dtheta1; accelerations(2)]);

% A single ordered parameter vector provides a stable simulation interface.
params = [I0; J1; m1; L0; l1; g; b0; b1; Rm; Kt; Ke];
plantFcn = matlabFunction(xdot, 'Vars', {x, u, params});

%% 5. Equation viewer -- generated from symbolic results
% Set false to build the model without opening a figure.
showDashboard = true;
if showDashboard
    % Re-running the script refreshes the viewer rather than duplicating it.
    old = findall(groot, 'Type', 'figure', 'Tag', 'FurutaModelViewer');
    if ~isempty(old)
        delete(old);
    end

    fig = figure('Name', 'Furuta | Symbolic Model', ...
        'Tag', 'FurutaModelViewer', 'NumberTitle', 'off', ...
        'Color', 'w', 'Position', [180 150 1100 540]);

    tabs = uitabgroup(fig, 'Units', 'normalized', ...
        'Position', [0.015 0.03 0.97 0.94]);
    tabEnergy = uitab(tabs, 'Title', 'Energy', 'BackgroundColor', 'w');
    tabDynamics = uitab(tabs, 'Title', 'Mechanical Equations', 'BackgroundColor', 'w');
    tabMotor = uitab(tabs, 'Title', 'Motor & States', 'BackgroundColor', 'w');

    % Energy equations are read directly from the expressions above.
    ax = viewerAxes(tabEnergy);
    viewerLabel(ax, .89, 'Energy and Lagrangian', 17);
    viewerLabel(ax, .76, 'Kinetic energy T', 11);
    viewerMath(ax, .66, ['T=' prettyLatex(T)], 12);
    viewerLabel(ax, .50, 'Potential energy V', 11);
    viewerMath(ax, .42, ['V=' prettyLatex(V)], 14);
    viewerLabel(ax, .28, 'Lagrangian L = T - V', 11);
    viewerMath(ax, .17, ['\mathcal{L}=' prettyLatex(Lagr)], 11);

    % Friction and motor substitutions are included automatically.
    ax = viewerAxes(tabDynamics);
    viewerLabel(ax, .89, 'Nonlinear equations of motion', 17);
    viewerLabel(ax, .76, 'Rotary arm', 11);
    viewerMath(ax, .65, prettyLatex(voltageEqs(1)), 12);
    viewerLabel(ax, .49, 'Passive pendulum', 11);
    viewerMath(ax, .38, prettyLatex(voltageEqs(2)), 12);
    viewerLabel(ax, .21, 'Both equations include viscous friction', 11);

    ax = viewerAxes(tabMotor);
    viewerLabel(ax, .89, 'Motor and state definition', 17);
    viewerLabel(ax, .76, 'Voltage relation (inductance neglected)', 11);
    viewerMath(ax, .67, prettyLatex(motorVoltageEq), 14);
    viewerLabel(ax, .54, 'Motor torque', 11);
    viewerMath(ax, .46, prettyLatex(motorTorqueEq), 14);
    viewerLabel(ax, .33, 'Voltage-to-torque substitution', 11);
    viewerMath(ax, .25, ['\tau=' prettyLatex(torqueExpr)], 13);
    viewerLabel(ax, .13, 'States: theta0, dtheta0, theta1, dtheta1', 11);
end

%% Local visualization helpers (MATLAB requires functions at script end)
function ax = viewerAxes(tab)
    ax = axes('Parent', tab, 'Position', [0.04 0.05 0.92 0.90], ...
        'Visible', 'off', 'Color', 'w');
    axis(ax, [0 1 0 1]);
end

function viewerLabel(ax, y, labelText, fontSize)
    text(ax, .02, y, labelText, 'Units', 'normalized', ...
        'FontSize', fontSize, 'FontWeight', 'bold', 'Color', 'k', ...
        'Interpreter', 'none', 'VerticalAlignment', 'middle');
end

function viewerMath(ax, y, body, fontSize)
    % Reduce type size on long expressions; the source remains symbolic.
    if numel(body) > 350
        fontSize = min(fontSize, 9);
    elseif numel(body) > 240
        fontSize = min(fontSize, 10);
    elseif numel(body) > 160
        fontSize = min(fontSize, 11);
    end
    text(ax, .02, y, ['$$' body '$$'], ...
        'Units', 'normalized', 'Interpreter', 'latex', ...
        'FontSize', fontSize, 'Color', 'k', ...
        'VerticalAlignment', 'middle');
end

function out = prettyLatex(expr)
    % Convert a symbolic equation to LaTeX, formatting variable names only.
    out = latex(expr);
    names = {'ddtheta0','ddtheta1','dtheta0','dtheta1', ...
             'theta0','theta1','I0','J1','m1','L0','l1', ...
             'b0','b1','Rm','Kt','Ke','Vm','im','tau'};
    symbols = {'\ddot{\theta}_{0}','\ddot{\theta}_{1}', ...
               '\dot{\theta}_{0}','\dot{\theta}_{1}', ...
               '\theta_{0}','\theta_{1}','I_{0}','J_{1}', ...
               'm_{1}','L_{0}','l_{1}','b_{0}','b_{1}', ...
               'R_{m}','K_{t}','K_{e}','V_{m}','i_{m}','\tau'};
    % Placeholders avoid accidentally replacing text inside LaTeX markup.
    for n = 1:numel(names)
        name = names{n};
        marker = sprintf('ZZMARK%02dZZ',n);
        if isstrprop(name(end),'digit')
            base = name(1:end-1); digit = name(end);
            out = strrep(out, ['\mathrm{' base '}_{' digit '}'], marker);
            out = strrep(out, ['\mathit{' base '}_{' digit '}'], marker);
        end
        out = strrep(out, ['\mathrm{' name '}'], marker);
        out = strrep(out, ['\mathit{' name '}'], marker);
        out = strrep(out, ['\operatorname{' name '}'], marker);
        % Replace plain names only when not part of another identifier.
        pat = ['(?<![A-Za-z\\])' regexptranslate('escape',name) '(?![A-Za-z0-9])'];
        out = regexprep(out, pat, marker);
    end
    for n = 1:numel(names)
        out = strrep(out, sprintf('ZZMARK%02dZZ',n), symbols{n});
    end
end
