%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_s, u_s] = compute_steady_state(params, d)

% unconstrained (test function does nontake feasibility into account!)
%disp('no constraints')
A_ss = [eye(params.model.nx) - params.model.A, -params.model.B;
    params.model.C_ref*params.model.C, zeros(size(params.exercise.T_ref,1),params.model.nu)];

b_ss = [params.model.Bd*d  ;
    params.exercise.T_ref - params.model.C_ref*params.model.Cd*d];

sol = A_ss\b_ss;
x_s = sol(1:params.model.nx);
u_s = sol(1+params.model.nx:params.model.nu+params.model.nx);


% disp('has constraints')
% nx = params.model.nx;
% nu = params.model.nu;
% 
% C1 = params.model.C_ref*params.model.C;
% C2 = params.exercise.T_ref - params.model.C_ref*params.model.Cd*d;
% 
% H = zeros(nx + nu, nx + nu);
% f = zeros(nx + nu, 1);
% 
% H(1:nx, 1:nx) = 2 * (C1'*C1);
% f(1:nx,1) = transpose(-2 * (C2'*C1));
% 
% Aeq = [eye(params.model.nx) - params.model.A, -params.model.B];
% beq = params.model.Bd*d;
% 
% Hx = params.constraints.StateMatrix;
% Hu = params.constraints.InputMatrix;
% hx = params.constraints.StateRHS;
% hu = params.constraints.InputRHS;
% 
% A = [Hx, zeros(size(Hx,1), nu);
%     zeros(size(Hu,1),nx), Hu];
% b = [hx;hu];
% 
% sol = quadprog(H,f,A,b,Aeq,beq);
% 
% x_s = sol(1:params.model.nx);
% u_s = sol(1+params.model.nx:params.model.nu+params.model.nx);

end
