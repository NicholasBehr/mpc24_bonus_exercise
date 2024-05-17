%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ADD STUFF HERE
params = generate_params_cc();
params_aug_obs = generate_params_aug_obs(params);

x0 = params.exercise.InitialConditionA;
radiation = params.exercise.RadiationB;
d = [params.model.a1o; params.model.a2o; params.model.a3o] * ...
    params.exercise.To + radiation;
Disturbances = ones(params.model.nd,params.exercise.SimHorizon+1).*d;

x0_est = params.exercise.x0_est;
d0_est = params.exercise.d0_est;

% cost matrices
Q = eye(params_aug_obs.model.nx) * 15;
R = eye(params_aug_obs.model.ny);

% compute feedback matrix and polesx
A_obs = params_aug_obs.model.A;
C_obs = params_aug_obs.model.C;

[~,K,poles] = idare(A_obs',C_obs',Q,R);
L = -K';
poles = poles';

obsv = Linear_Observer(L, params_aug_obs);
ctrl = Controller_constant();
[X, U, X_est, D_est, ctrl_info] = simulate_observer(x0, x0_est, d0_est, ctrl, obsv, Disturbances, params);


%% Save
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "obsv_tuning_script_cc.mat"), 'poles', 'X', 'X_est');
