
function model = linearize_upright(showPlot)
%LINEARIZE_UPRIGHT Linearize Furuta dynamics around upright.
%
% Method: First-order Taylor expansion (Jacobian).
%
% Equilibrium:
%   theta0 = 0, theta0dot = 0
%   theta1 = 0, theta1dot = 0
%   Vm = 0
%
% Output:
%   model - Structure containing A, B, C, D, sys,
%           equilibrium, parameters and eigenvalues.

if nargin < 1
    showPlot = true;
end

%% 1. Load Symbolic Mathematical Model
modelFolder = fileparts(mfilename('fullpath'));
addpath(modelFolder);

run(fullfile(modelFolder, 'symbolic_lagrangian.m'));

%% 2. Load Shared Physical Parameters
[p, pNum, parameterMode] = furuta_parameters(params);

fprintf('\nParameter source: %s\n', parameterMode);

%% 3. Define Upright Equilibrium
% theta1 = 0 is upright.
% Both angular velocities and motor voltage are zero.

x_eq = [0; 0; 0; 0];
u_eq = 0;

% Verify equilibrium: f(x_eq, u_eq) = 0
f_eq = double(subs(xdot, ...
    [x; u; params], [x_eq; u_eq; pNum]));

assert(norm(f_eq, inf) < 1e-9, ...
    'The selected operating point is not an equilibrium.');

%% 4. Linearize Using Jacobians
% Nonlinear model: xdot = f(x,u)
%
% First-order Taylor linearization:
% delta_xdot = A*delta_x + B*delta_u
%
% A = df/dx evaluated at equilibrium
% B = df/du evaluated at equilibrium

A_sym = jacobian(xdot, x);
B_sym = jacobian(xdot, u);

A_up = double(subs(A_sym, ...
    [x; u; params], [x_eq; u_eq; pNum]));

B_up = double(subs(B_sym, ...
    [x; u; params], [x_eq; u_eq; pNum]));

%% 5. Define Output Matrices
% Outputs: arm angle and pendulum angle.

C_up = [1 0 0 0;
        0 0 1 0];

D_up = zeros(2,1);

% MATLAB continuous-time state-space model
sys_up = ss(A_up, B_up, C_up, D_up);

%% 6. Open-Loop Stability Analysis
eigenvalues = eig(A_up);

controllabilityMatrix = ctrb(A_up, B_up);
observabilityMatrix   = obsv(A_up, C_up);

rankCo = rank(controllabilityMatrix);
rankOb = rank(observabilityMatrix);

fprintf('\n===== UPRIGHT STABILITY ANALYSIS =====\n');

fprintf('\nState matrix A:\n');
disp(A_up);

fprintf('Input matrix B:\n');
disp(B_up);

fprintf('Open-loop eigenvalues:\n');
disp(eigenvalues);

fprintf('Controllability rank: %d / 4\n', rankCo);
fprintf('Observability rank:   %d / 4\n', rankOb);

fprintf('\nPole characteristics:\n');
damp(sys_up);

if any(real(eigenvalues) > 1e-8)
    fprintf('\nResult: Upright equilibrium is UNSTABLE.\n');
else
    fprintf('\nNo positive-real-part poles detected.\n');
    fprintf('Further stability classification may be required.\n');
end

%% 7. Save Results for Controller Design
model.A = A_up;
model.B = B_up;
model.C = C_up;
model.D = D_up;

model.sys = sys_up;
model.eigenvalues = eigenvalues;
model.controllabilityRank = rankCo;
model.observabilityRank = rankOb;

model.x_eq = x_eq;
model.u_eq = u_eq;

model.parameters = p;
model.pNum = pNum;
model.parameterMode = parameterMode;

%% 8. S-Plane Visualization
if showPlot
    fig = figure('Name', 'Furuta | Upright S-Plane');
    theme(fig, "light");

    pzp = pzplot(sys_up);
    pzp.AxesStyle.GridVisible = "on";

    title('Open-Loop Pole-Zero Map - Upright Equilibrium');
end

end
