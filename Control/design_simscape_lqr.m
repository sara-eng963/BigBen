
%% Furuta Pendulum - Simscape LQR Controller
% Run after analyze_simscape_linearization.m
%
% States:
% x = [theta0; dtheta0; theta1; dtheta1]
%
% Controller:
% Vm = -K_lqr_sim * x

%% 1. Verify the linearized plant

assert(exist('A','var') == 1 && exist('B','var') == 1, ...
    'Run Simscape linearization analysis first.');

assert(isequal(size(A), [4 4]) && ...
       isequal(size(B), [4 1]), ...
    'Expected four states and one voltage input.');

assert(rank(ctrb(A,B)) == 4, ...
    'The plant is not fully controllable.');

%% 2. Choose initial LQR weights
% Preliminary design scales, NOT verified hardware limits.
% They must be justified and tuned using simulations.

armAngleScale  = deg2rad(30);
armSpeedScale  = 1.0;          % rad/s
pendAngleScale = deg2rad(5);
pendSpeedScale = 1.0;          % rad/s
voltageScale   = 5.0;          % V, preliminary scale

stateScales = [armAngleScale;
               armSpeedScale;
               pendAngleScale;
               pendSpeedScale];

% Bryson's rule
Q_lqr_sim = diag([1, 0.1, 100, 10]);
R_lqr_sim = 100;

%% 3. Calculate LQR gain

[K_lqr_sim, S_lqr_sim, P_lqr_sim] = ...
    lqr(A, B, Q_lqr_sim, R_lqr_sim);

%% 4. Verify closed-loop stability

Acl_lqr_sim = A - B*K_lqr_sim;

actualPoles = eig(Acl_lqr_sim);

assert(all(real(actualPoles) < 0), ...
    'LQR closed-loop system is unstable.');

%% 5. Display controller results

fprintf('\n===== SIMSCAPE LQR =====\n');

disp('Q matrix:');
disp(Q_lqr_sim);

disp('R value:');
disp(R_lqr_sim);

disp('LQR gain K:');
disp(K_lqr_sim);

disp('Closed-loop eigenvalues:');
disp(actualPoles);

%% 6. Check initial voltage demand
% Test example: pendulum 5 degrees away from upright.

x0_test = [0; 0; deg2rad(5); 0];

Vm_initial = -K_lqr_sim*x0_test;

fprintf('Initial voltage command: %.3f V\n', ...
        Vm_initial);

fprintf('Linear LQR stability check: PASSED\n');
fprintf('Nonlinear Simscape response: NOT YET VERIFIED\n');
