%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TE(Q,R,N,params)
            u = sdpvar(repmat(params.model.nu,1,N),repmat(1,1,N));
            x = sdpvar(repmat(params.model.nx,1,N+1),repmat(1,1,N+1));
            
            constraints = [];
            objective = 0;
            
            Hx = params.constraints.StateMatrix;
            hx = params.constraints.StateRHS;
            Hu = params.constraints.InputMatrix;
            hu = params.constraints.InputRHS;
                       
            for k = 1:N
                objective = objective + x{k}'*Q*x{k} + u{k}'*R*u{k};
                constraints = [constraints, x{k+1} == params.model.A*x{k} + params.model.B*u{k}];
                constraints = [constraints, Hx * x{k} <= hx, Hu * u{k} <= hu];
            end
            constraints = [constraints, norm(x{N+1}, 1) == 0];

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,x{1},{u{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end