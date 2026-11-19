
%% Pole Placement Controller Design
% Furuta Pendulum - Upright Stabilization
%
% Method: Full-State Feedback
% Reference: MCT414 Lecture 4 - State Feedback
%
% Control law: Vm = -K*x
% Closed-loop: xdot = (A-B*K)*x

clear;
clc;

% Locate shared modelling functions
controlFolder = fileparts(mfilename('fullpath'));
projectFolder = fileparts(controlFolder);

addpath(fullfile(projectFolder, 'Modeling'));

%% 1. Load Upright Linearized Model
% Reuse the existing mathematical model.

model = linearize_upright(false);

A_pp = model.A;
B_pp = model.B;

n = size(A_pp, 1);

%% 2. Check Controllability
% Arbitrary pole placement requires controllability.

Co = ctrb(A_pp, B_pp);

assert(rank(Co) == n, ...
    'System is not fully controllable.');

fprintf('Controllability: %d/%d\n', rank(Co), n);

%% 3. Desired Closed-Loop Specifications
% Based initially on the Lecture 4 example.

Ts = 2;           % Settling time [s]
zeta = 0.5;       % Damping ratio

% Second-order dominant pole calculations
omega_n = 4/(zeta*Ts);
omega_d = omega_n*sqrt(1-zeta^2);

% Dominant complex-conjugate poles
p1 = -zeta*omega_n + 1i*omega_d;
p2 = -zeta*omega_n - 1i*omega_d;

% Additional non-dominant poles
% Chosen farther left than the dominant poles
p3 = -20*zeta*omega_n;
p4 = -25*zeta*omega_n;

desiredPoles = [p1 p2 p3 p4];

fprintf('\n===== DESIGN SPECIFICATIONS =====\n');
fprintf('Settling time Ts = %.2f s\n', Ts);
fprintf('Damping ratio   = %.3f\n', zeta);
fprintf('Natural freq wn = %.3f rad/s\n', omega_n);
fprintf('Damped freq wd  = %.3f rad/s\n', omega_d);

fprintf('\nDesired poles:\n');
disp(desiredPoles.');

%% 4. Calculate State Feedback Gain
% Control law: Vm = -K*x
% MATLAB uses pole placement to obtain K.

K_pp = place(A_pp, B_pp, desiredPoles);

fprintf('\n===== FEEDBACK GAIN K =====\n');
disp(K_pp);

%% 5. Verify Closed-Loop Stability
% Closed-loop state matrix: Acl = A-BK

Acl_pp = A_pp - B_pp*K_pp;

actualPoles = eig(Acl_pp);

fprintf('\nActual closed-loop poles:\n');
disp(actualPoles);

assert(all(real(actualPoles) < 0), ...
    'Closed-loop system is not asymptotically stable.');

fprintf('Closed-loop eigenvalue test: PASSED\n');

%% 6. Prepare Simulink Variables
% The plant will output all 4 states for feedback.

C_pp = eye(n);
D_pp = zeros(n, 1);

% Initial condition: 1 degree away from upright
x0_pp = [0; 0; deg2rad(1); 0];

% Calculate initial voltage demand
u0_pp = -K_pp*x0_pp;

fprintf('\nInitial motor voltage = %.3f V\n', u0_pp);
fprintf('Note: Actuator voltage limits are not yet modelled.\n');

%% 7. Summary
fprintf('\n===== POLE PLACEMENT SUMMARY =====\n');
fprintf('Number of states: %d\n', n);
fprintf('Feedback gain K:\n');
disp(K_pp);

fprintf('Desired settling time: %.2f s\n', Ts);
fprintf('Linear closed-loop stability: VERIFIED\n');
fprintf('Nonlinear simulation: NOT YET VERIFIED\n');
