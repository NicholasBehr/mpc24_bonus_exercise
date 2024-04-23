%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_s, u_s] = compute_steady_state(params, d)

A_ss = [eye(params.model.nx) - params.model.A   ,  -params.model.B                       ;
    params.model.C_ref*params.model.C                         ,  zeros(size(params.exercise.T_ref,1),params.model.nu)];

b_ss = [params.model.Bd*d  ;
    params.exercise.T_ref - params.model.C_ref*params.model.Cd*d];

sol = A_ss\b_ss;
x_s = sol(1:params.model.nx);
u_s = sol(1+params.model.nx:params.model.nu+params.model.nx);

end
