%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [T1_max, T2_min, T2_max, P1_min, P1_max, P2_min, P2_max, input_cost, cstr_viol] = traj_constraints_cc(X, U, params)

    T1_max = max(X(1,:));
    T2_min = min(X(2,:));
    T2_max = max(X(2,:));
    P1_min = min(U(1,:));
    P2_max = max(U(2,:));
    P2_min = min(U(2,:));
    P1_max = max(U(1,:));

    u_max = [P1_max; P2_max];
    u_min = [P1_min; P2_min];
    x_max = [T1_max; T2_max; 0];
    x_min = [0 ; T2_min; 0];
    
    input_cost = 0;
    for Ut = U
        input_cost = input_cost + Ut'*Ut;
    end

    [H_u, h_u, H_x, h_x] = generate_constraints_cc(params);
    cstr_viol = (H_u*u_max > h_u) & (H_u*u_min > h_u);
    cstr_viol = cstr_viol & (H_x*x_max > h_x) & (H_x*x_min > h_x);
end

