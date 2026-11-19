
function [p, pNum, parameterMode] = furuta_parameters(symbolicParams)
%FURUTA_PARAMETERS Shared physical parameters for the Furuta pendulum.
%
% Inputs:
%   symbolicParams - Parameter vector from symbolic_lagrangian.m
%
% Outputs:
%   p             - Named parameter structure
%   pNum          - Numerical vector in the symbolic model's order
%   parameterMode - Selected parameter source
%
% All models must use this function.
% Do not duplicate parameter values in other scripts.

%% 1. Parameter Mode
parameterMode = "test";  % "test" | "hardware"

%% 2. Physical Parameters
switch parameterMode

    case "test"
        % Preliminary literature-based test configuration
        % Reference: Cazzolato & Prime (2011),
        % "On the Dynamics of the Furuta Pendulum"
        % DOI: 10.1155/2011/528341
        %
        % Uses the parameter mapping and modelling assumptions
        % currently adopted in run_open_loop.m.
        % NOT measurements of our actual hardware.

        % Mechanical parameters
        p.J0  = 2.48e-2 + 0.300*(0.150)^2;
        p.J1r = 3.86e-3;
        p.J1t = 3.86e-3;
        p.J1a = 0;         % Neglected longitudinal inertia

        p.m1 = 0.075;      % kg
        p.L0 = 0.278;      % m
        p.l1 = 0.148;      % m
        p.g  = 9.81;       % m/s^2

        % Viscous damping [N*m*s/rad]
        p.b0 = 1.00e-2;
        p.b1 = 2.80e-2;

        % Simplified DC motor
        p.Rm = 7.80;       % ohm
        p.Kt = 0.090;      % N*m/A
        p.Ke = 0.090;      % V*s/rad

        % Motor inductance is neglected in our 4-state model.

    case "hardware"
        % Replace with measured/CAD-derived parameters.
        % NaN prevents accidentally using unverified values.

        p.J0  = NaN;
        p.J1r = NaN;
        p.J1t = NaN;
        p.J1a = NaN;

        p.m1 = NaN;
        p.L0 = NaN;
        p.l1 = NaN;
        p.g  = 9.81;

        p.b0 = NaN;
        p.b1 = NaN;

        p.Rm = NaN;
        p.Kt = NaN;
        p.Ke = NaN;

    otherwise
        error('Invalid parameter mode.');
end

%% 3. Build Parameter Vector
% Read parameter names directly from symbolic_lagrangian.m.
% This avoids duplicating the parameter ordering.

pNum = zeros(numel(symbolicParams), 1);

for k = 1:numel(symbolicParams)
    name = char(symbolicParams(k));

    if ~isfield(p, name)
        error('Missing parameter in furuta_parameters: %s', name);
    end

    pNum(k) = p.(name);
end

%% 4. Validate Parameters
if any(~isfinite(pNum)) || ~isreal(pNum)
    error('Missing or invalid parameters in mode: %s', ...
          parameterMode);
end

positiveParams = [p.J0, p.J1r, p.J1t, p.m1, ...
                  p.L0, p.l1, p.g, p.Rm, p.Kt, p.Ke];

nonnegativeParams = [p.J1a, p.b0, p.b1];

if any(positiveParams <= 0) || any(nonnegativeParams < 0)
    error('Nonphysical Furuta parameter values.');
end

end
