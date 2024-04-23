%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x,u,ctrl_info] = simulate(x0, ctrl, params)
    
    q = ones(1,params.model.nx);
    r = ones(1,params.model.nu);

    Q = diag(q);
    R = diag(r);
    x = zeros(params.model.nx,params.exercise.SimHorizon+1);
    x(:,1) = x0;
    u = zeros(params.model.nu,params.exercise.SimHorizon);
    for k=1:params.exercise.SimHorizon
        [u(:,k), ctrl_info(k)] = ctrl.eval(x(:,k));
        x(:,k+1) = params.model.A*x(:,k) + params.model.B*u(:,k);
    end

end