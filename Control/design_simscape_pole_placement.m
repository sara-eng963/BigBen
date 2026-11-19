
%% Furuta Pendulum - Simscape Pole Placement
% Uses the state-space matrices obtained from Model Linearizer.
% Requires A and B from analyze_simscape_linearization.m.
%
% State order:
% x = [theta0; dtheta0; theta1; dtheta1]
%
% Control law:
% Vm = -K_pp_sim*x

%% 1. Verify the linearized plant
assert(exist('A','var') == 1 && exist('B','var') == 1, ...
    'Run the Simscape linearization analysis first.');

n = size(A,1);

assert(rank(ctrb(A,B)) == n, ...
    'The system is not fully controllable.');

%% 2. Desired closed-loop specifications
Ts = 2;          % Settling time [s]
zeta = 0.5;      % Damping ratio

omega_n = 4/(zeta*Ts);
omega_d = omega_n*sqrt(1-zeta^2);

% Dominant poles
p1 = -zeta*omega_n + 1i*omega_d;
p2 = -zeta*omega_n - 1i*omega_d;

% Additional non-dominant poles
p3 = -20*zeta*omega_n;
p4 = -25*zeta*omega_n;

desiredPoles = [p1 p2 p3 p4];

fprintf('\n===== DESIRED POLES =====\n');
disp(desiredPoles.');

%% 3. Calculate feedback gain
K_pp_sim = place(A, B, desiredPoles);

fprintf('\n===== STATE-FEEDBACK GAIN =====\n');
disp(K_pp_sim);

%% 4. Verify closed-loop stability
Acl_pp_sim = A - B*K_pp_sim;
closedLoopPoles = eig(Acl_pp_sim);

fprintf('\n===== CLOSED-LOOP EIGENVALUES =====\n');
disp(closedLoopPoles);

assert(all(real(closedLoopPoles) < 0), ...
    'Closed-loop poles are not all stable.');

fprintf('Linear pole-placement verification: PASSED\n');
fprintf('Nonlinear Simscape simulation: NOT YET TESTED\n');
