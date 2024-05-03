%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% CONTINUE BELOW THIS LINE

params = generate_params_cc();
params_delta = generate_params_delta_cc(params);

q = [10, 100 , 0];
Q = diag(q);
r = [1.2e-5 , 1.8e-5];
R = diag(r);

Controller = LQR(Q, R, params_delta);
[X, U, ctrl_info] = simulate(params_delta.exercise.InitialConditionA, Controller, params_delta);

% [X_abs, U_abs] = traj_delta2abs(X, U, params_delta.exercise.x_s, params_delta.exercise.u_s);
plot_trajectory_cc(X, U, ctrl_info, params_delta);

%% Save results
current_folder = fileparts(which(mfilename));
save(fullfile(current_folder, "lqr_tuning_script_cc.mat"), 'Q', 'R', 'X', 'U');