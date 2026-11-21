% Load the actual BigBen plant and prepare all three controller modes.
bigBenRoot = fileparts(mfilename('fullpath'));
addpath(bigBenRoot, fullfile(bigBenRoot,'Modeling'), ...
    fullfile(bigBenRoot,'Modeling','Subsystems'), ...
    fullfile(bigBenRoot,'Control'), fullfile(bigBenRoot,'URDF'));

bigBenData = load(fullfile(bigBenRoot,'Modeling','Linearization','linsys2.mat'), ...
    'linsys2','plantEvidence');
bigBenUpright = bigBenData.plantEvidence.operatingOutputs(3);
[A,B,motor,K_pp_sim,K_lqr_sim,pidGains] = prepareBigBen(bigBenData.linsys2);
fprintf('\nBigBen gains loaded. Selector: 1 PID, 2 pole placement, 3 LQR, 4 zero volts.\n');
fprintf('Raw pendulum upright: %.9g rad. Set a release angle relative to this value.\n', bigBenUpright);

function [A,B,motor,K_pp_sim,K_lqr_sim,pidGains] = prepareBigBen(linsys1)
    % The existing analysis expects this variable name for its input.
    analyze_simscape_linearization
    design_simscape_pole_placement
    design_simscape_lqr
    design_simscape_pid
end
