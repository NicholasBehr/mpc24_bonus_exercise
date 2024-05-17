%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X, U, X_est, D_est, ctrl_info] = simulate_observer(x0, x0_est, d0_est, ctrl, obsv, Disturbances, params)
    
    X_est = zeros(params.model.nx, params.exercise.SimHorizon+1);
    X = zeros(params.model.nx, params.exercise.SimHorizon+1);
    D_est = zeros(params.model.nd, params.exercise.SimHorizon+1);
    U = zeros(params.model.nu, params.exercise.SimHorizon);

    X_est(:,1) = x0_est';
    X(:,1) = x0;
    D_est(:,1) = d0_est';


    for k=1:params.exercise.SimHorizon
        % Step 1
        [x_s, u_s] = compute_steady_state(params, D_est(:,k));
        
        % Step 2
        [U(:,k), ctrl_info(k)] = ctrl.eval(X_est(:,k), D_est(:,k), x_s, u_s);

        % Step 3
        X(:,k+1) = params.model.A*X(:,k) + params.model.B*U(:,k) + params.model.Bd*Disturbances(:,k);
        
        % Measurement

        % Step 4
        y = params.model.C*X(:,k+1) + params.model.Cd*Disturbances(:,k+1);
        X_aug = obsv.eval([X_est(:,k);D_est(:,k)], U(:,k), y);
        X_est(:,k+1) = X_aug(1:params.model.nx);
        D_est(:,k+1) = X_aug(params.model.nx+1:end);
    end
    
end
