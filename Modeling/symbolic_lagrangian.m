
%% 1. Symbolic Variables
clear; clc;

% Generalized coordinates (angles)
syms theta0 theta1 real

% Angular velocities
syms dtheta0 dtheta1 real

% Angular accelerations
syms ddtheta0 ddtheta1 real

% Physical parameters
syms I0 J1 m1 L0 l1 g tau real

% State vectors
q   = [theta0; theta1];
dq  = [dtheta0; dtheta1];
ddq = [ddtheta0; ddtheta1];


%% 2. Energy Equations

% Rotary arm kinetic energy
T_arm = 1/2 * I0 * dtheta0^2;

% Pendulum rotational kinetic energy
T_pend_rot = 1/2 * J1 * dtheta1^2;

% Pendulum COM velocity (tutorial approximation)
vx = L0*dtheta0 + l1*dtheta1*cos(theta1);
vy = l1*dtheta1*sin(theta1);

% Pendulum translational kinetic energy
T_pend_trans = 1/2 * m1 * (vx^2 + vy^2);

% Total kinetic energy
T = simplify(T_arm + T_pend_rot + T_pend_trans);

% Potential energy
V = m1*g*l1*cos(theta1);

% Lagrangian
Lagr = simplify(T - V);

%% Display Results

disp('Total Kinetic Energy T:')
disp(T)

disp('Potential Energy V:')
disp(V)

disp('Lagrangian L = T - V:')
disp(Lagr)


%% 3. Euler-Lagrange Derivation

% Generalized forces: motor torque, unactuated pendulum
Q = [tau; 0];

% Initialize equations of motion
EOM = sym(zeros(2,1));

for i = 1:2

    % Derivative with respect to angular velocity
    dL_ddq = diff(Lagr, dq(i));

    % Total time derivative (chain rule)
    dt_term = jacobian(dL_ddq, q)*dq ...
            + jacobian(dL_ddq, dq)*ddq;

    % Derivative with respect to joint angle
    dL_dq = diff(Lagr, q(i));

    % Euler-Lagrange equation
    EOM(i) = simplify(dt_term - dL_dq);

end

% Equations of motion
equations = EOM == Q;


%% 4. LaTeX Equation Viewer


%% Format Symbolic Equations as LaTeX

eqTex = cell(2,1);

% Variable names and their mathematical notation
names = {'ddtheta0', 'ddtheta1', 'dtheta0', 'dtheta1'};

symbols = {'\ddot{\theta}_{0}', '\ddot{\theta}_{1}', ...
           '\dot{\theta}_{0}',  '\dot{\theta}_{1}'};

for i = 1:2
    eqTex{i} = latex(equations(i));

    for j = 1:numel(names)
        name = names{j};
        formatted = symbols{j};

        % MATLAB may convert trailing digits into subscripts
        base = name(1:end-1);
        digit = name(end);

        eqTex{i} = strrep(eqTex{i}, ...
            ['\mathrm{' base '}_{' digit '}'], formatted);

        eqTex{i} = strrep(eqTex{i}, ...
            ['\mathrm{' name '}'], formatted);

        eqTex{i} = strrep(eqTex{i}, name, formatted);
    end
end


% Create a dedicated mathematical display window
fig = figure( ...
    'Name', 'Furuta Pendulum - Nonlinear Dynamics', ...
    'Color', 'w', ...
    'Position', [100 100 1350 450]);

ax = axes(fig, ...
    'Position', [0.04 0.05 0.92 0.90], ...
    'Visible', 'off');

axis(ax, [0 1 0 1]);

% Window title
text(ax, 0.02, 0.92, 'Nonlinear Equations of Motion', ...
    'FontSize', 18, 'FontWeight', 'bold');

% Equation 1 - Rotary arm
text(ax, 0.02, 0.75, '1. Rotary Arm', ...
    'FontSize', 14, 'FontWeight', 'bold');

text(ax, 0.02, 0.61, ['$$' eqTex{1} '$$'], ...
    'Interpreter', 'latex', 'FontSize', 15);

% Equation 2 - Pendulum
text(ax, 0.02, 0.40, '2. Pendulum', ...
    'FontSize', 14, 'FontWeight', 'bold');

text(ax, 0.02, 0.26, ['$$' eqTex{2} '$$'], ...
    'Interpreter', 'latex', 'FontSize', 15);

set(findall(fig, 'Type', 'text'), 'Color', 'k');