%% BigBen - PID with integral action on arm-position error
% q1 is the measured pendulum angle relative to upright. The referenced
% subsystem subtracts bigBenUpright from the raw plant signal.
% The four P/D terms sum with KiArm*z, where z_dot = -theta0.
% Integrating pendulum error can leave a neutral arm mode on this plant.
%% 1. Check the actual upright linearization
clear pidGains
pidRepo = fileparts(fileparts(mfilename('fullpath')));
pidLinearization = fullfile(pidRepo,'Modeling','Linearization','linsys2.mat');
assert(isfile(pidLinearization),'BigBenPID:MissingPlant', ...
    'Export the verified actual BigBen voltage-input plant as linsys2 to %s.', ...
    pidLinearization);

pidData = load(pidLinearization,'linsys2');
assert(isfield(pidData,'linsys2') && isa(pidData.linsys2,'ss'), ...
    'BigBenPID:InvalidPlant','linsys2 must be a state-space model.');
pidPlant = pidData.linsys2;
[pidAraw,pidBraw,pidCraw,pidDraw] = ssdata(pidPlant);
assert(pidPlant.Ts == 0 && ~hasdelay(pidPlant) && ...
    isequal(size(pidAraw),[4 4]) && isequal(size(pidBraw),[4 1]) && ...
    isequal(size(pidCraw),[4 4]) && isequal(size(pidDraw),[4 1]), ...
    'BigBenPID:InvalidPlant', ...
    'Expected one continuous, delay-free, four-state model with one input and four outputs.');
assert(strcmpi(pidPlant.InputName{1},'Voltage'), ...
    'BigBenPID:InvalidPlant','Verify the motor-voltage input in V and name it Voltage.');

pidStateNames = ["theta0"; "dtheta0"; "theta1"; "dtheta1"];
[pidHasState,pidOutputIdx] = ismember(pidStateNames,string(pidPlant.OutputName));
assert(all(pidHasState) && norm(pidDraw,'fro') < 1e-10, ...
    'BigBenPID:InvalidPlant','Expected four named state outputs with negligible input feedthrough.');

% Same output-coordinate transformation as analyze_simscape_linearization.m.
pidT = pidCraw(pidOutputIdx,:);
assert(rank(pidT) == 4,'BigBenPID:InvalidPlant','State outputs must be independent.');
pidA = pidT*pidAraw/pidT;
pidB = pidT*pidBraw;
pidOpenPoles = eig(pidA);
fprintf('\n===== BIGBEN ACTUAL PLANT OPEN-LOOP POLES =====\n');
disp(pidOpenPoles);
assert(any(real(pidOpenPoles) > 1e-8) && rank(ctrb(pidA,pidB)) == 4 && ...
    rank(obsv(pidA,[1 0 0 0; 0 0 1 0])) == 4, ...
    'BigBenPID:InvalidPlant', ...
    'Verify upright instability, controllability and observability from the two angles.');

%% 2. Design and screen the PID gains
pidTracking = [pidA, zeros(4,1); -1, 0, 0, 0, 0];
pidInput = [pidB; 0];
assert(rank(ctrb(pidTracking,pidInput)) == 5, 'BigBenPID:InvalidPlant', ...
    'The actual plant with arm integral action must be controllable.');

% Keep the pole-placement benchmark dynamics and add one slow integral pole.
pidTargets = [-2+sqrt(12)*1i, -2-sqrt(12)*1i, -40, -50, -1];
pidLegacyTrial = exist('pidTrial','var') && isstruct(pidTrial) && isscalar(pidTrial);
if pidLegacyTrial
    pidK = [pidTrial.KpArm pidTrial.KdArm pidTrial.KpPend pidTrial.KdPend 0];
    assert(pidTrial.KiPend == 0, 'BigBenPID:UnsupportedIntegral', ...
        'Pendulum integral is unsupported. Use the default for arm integral PID.');
    if isfield(pidTrial,'KiArm')
        pidK(5) = -pidTrial.KiArm;
    end
else
    pidK = place(pidTracking,pidInput,pidTargets);
end
validateattributes(pidK,{'numeric'},{'real','finite','row','numel',5});
pidX0 = [0; 0; deg2rad(5); 0];
if pidK(5) == 0
    pidAcl = pidA - pidB*pidK(1:4);
    pidVoltage = -pidK(1:4);
else
    pidAcl = pidTracking - pidInput*pidK;
    pidVoltage = -pidK;
    pidX0 = [pidX0; 0];
end
pidClosedPoles = eig(pidAcl);
assert(all(real(pidClosedPoles) < -1e-8), 'BigBenPID:UnstableTrial', ...
    'Candidate has an unstable or neutral closed-loop pole.');

%% 3. Test a +5 degree release with the linearized plant
n = numel(pidX0);
pidOutputs = [eye(4) zeros(4,n-4); pidVoltage];
pidClosedLoop = ss(pidAcl,zeros(n,1),pidOutputs,zeros(5,1));
[pidResponse,pidTime] = initial(pidClosedLoop,pidX0,0:0.01:10);
assert(all(isfinite(pidResponse(:))), 'BigBenPID:InvalidResponse', ...
    'The linear response contains nonfinite values.');
fprintf('PID linear peak voltage %.3f V, peak arm excursion %.3f deg.\n', ...
    max(abs(pidResponse(:,5))),rad2deg(max(abs(pidResponse(:,1)))));
pidGains = struct('KpArm',pidK(1),'KdArm',pidK(2),'KpPend',pidK(3), ...
    'KdPend',pidK(4),'KiArm',-pidK(5));
disp(pidGains);
