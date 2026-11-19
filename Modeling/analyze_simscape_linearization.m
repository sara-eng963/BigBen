
%% SECTION 1 - DC Motor Parameters
% Independent electrical motor assumptions.
% Mechanical parameters are defined in the imported URDF.

motor.Rm = 8.0;     % Armature resistance [ohm]
motor.Kt = 0.05;    % Torque constant [N*m/A]
motor.Ke = 0.05;    % Back-EMF constant [V*s/rad]

fprintf('\n===== DC MOTOR PARAMETERS =====\n');
fprintf('Rm = %.4f ohm\n', motor.Rm);
fprintf('Kt = %.4f N*m/A\n', motor.Kt);
fprintf('Ke = %.4f V*s/rad\n', motor.Ke);


%% SECTION 2 - Open-Loop Simscape Linearization Analysis
% Prerequisite: Manually export the latest model as linsys1.
%
% Controller-independent state ordering:
% x = [theta0; dtheta0; theta1; dtheta1]

assert(exist('linsys1', 'var') == 1, ...
    'Export the latest model as linsys1 first.');

% Extract the Simscape linearization matrices
[A_sim, B_sim, C_sim, D_sim] = ssdata(linsys1);

fprintf('\n===== RAW SIMSCAPE MODEL =====\n');
fprintf('States = %d, Inputs = %d, Outputs = %d\n', ...
    size(A_sim,1), size(B_sim,2), size(C_sim,1));

disp('Raw A:'); disp(A_sim);
disp('Raw B:'); disp(B_sim);
disp('Output names:'); disp(string(linsys1.OutputName));

% Reorder measured states to our standard convention
requiredOutputs = [
    "theta0"
    "dtheta0"
    "theta1"
    "dtheta1"
];

[found, idx] = ismember( ...
    requiredOutputs, string(linsys1.OutputName));

assert(all(found), 'Required state outputs are missing.');
assert(isequal(size(A_sim), [4 4]), ...
    'Expected a four-state Simscape model.');
assert(isequal(size(B_sim), [4 1]), ...
    'Expected one motor-voltage input.');
assert(norm(D_sim, 'fro') < 1e-10, ...
    'Unexpected direct feedthrough in state measurements.');

% x = T*z, where z is the original Simscape state ordering
T = C_sim(idx, :);

assert(rank(T) == 4, ...
    'Measured outputs do not define four independent states.');

A = T*A_sim/T;
B = T*B_sim;
C = eye(4);
D = zeros(4,1);

sys_sim = ss(A, B, C, D);
sys_sim.InputName = {'Motor Voltage'};
sys_sim.OutputName = cellstr(requiredOutputs);

fprintf('\n===== REORDERED STATE-SPACE MODEL =====\n');
disp('x = [theta0; dtheta0; theta1; dtheta1]');
disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);

% Open-loop stability analysis
openLoopPoles = eig(A);

fprintf('\n===== OPEN-LOOP STABILITY =====\n');
disp('Open-loop eigenvalues:');
disp(openLoopPoles);

if any(real(openLoopPoles) > 1e-8)
    fprintf('Result: UNSTABLE upright equilibrium\n');
else
    fprintf('No right-half-plane poles detected.\n');
end

% Controllability
Co = ctrb(A, B);
rankCo = rank(Co);

fprintf('\n===== CONTROLLABILITY =====\n');
fprintf('Rank = %d / %d\n', rankCo, size(A,1));

% Observability using measured arm and pendulum positions
C_angles = [1 0 0 0;
            0 0 1 0];

rankOb = rank(obsv(A, C_angles));

fprintf('\n===== OBSERVABILITY =====\n');
fprintf('Rank = %d / %d\n', rankOb, size(A,1));

fprintf('\nOpen-loop Simscape analysis complete.\n');
