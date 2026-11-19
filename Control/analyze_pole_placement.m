
%% Pole Placement - Closed-Loop Performance Analysis
% Furuta Pendulum | Linear Stabilization Test

%% 1. Load Controller Design
design_pole_placement;

%% 2. Run Simulink Model
simOut = sim('pole_placement');

% Retrieve logged signals
xData = simOut.logsout.get('x_states').Values;
uData = simOut.logsout.get('Vm_cmd').Values;

t = xData.Time(:);
X = squeeze(xData.Data);

tu = uData.Time(:);
Vm = squeeze(uData.Data);
Vm = Vm(:);

assert(size(X,1) == numel(t) && size(X,2) == 4, ...
    'Expected N-by-4 state data.');

%% 3. Pendulum Stabilization Performance
% Initial-condition response, not a step response.
% lsiminfo is appropriate for this test.

pendulum = X(:,3);

info = lsiminfo(pendulum, t, 0, pendulum(1));

fprintf('\n===== PENDULUM PERFORMANCE =====\n');
fprintf('Settling time (2%%) = %.4f s\n', info.SettlingTime);
fprintf('Maximum deviation  = %.4f deg\n', ...
    max(abs(rad2deg(pendulum))));
fprintf('Final angle error  = %.6f deg\n', ...
    abs(rad2deg(pendulum(end))));

%% 4. Rotary Arm Performance
arm = X(:,1);

fprintf('\n===== ARM PERFORMANCE =====\n');
fprintf('Maximum arm angle  = %.4f deg\n', ...
    max(abs(rad2deg(arm))));
fprintf('Final arm angle    = %.6f deg\n', ...
    rad2deg(arm(end)));

%% 5. Motor Voltage Performance
peakVoltage = max(abs(Vm));
rmsVoltage = sqrt(trapz(tu, Vm.^2) / ...
                  (tu(end) - tu(1)));

fprintf('\n===== MOTOR VOLTAGE =====\n');
fprintf('Peak voltage       = %.4f V\n', peakVoltage);
fprintf('RMS voltage        = %.4f V\n', rmsVoltage);
fprintf('Initial voltage    = %.4f V\n', Vm(1));

%% 6. Closed-Loop Pole Verification
fprintf('\n===== CLOSED-LOOP POLES =====\n');
disp(eig(A_pp - B_pp*K_pp));
