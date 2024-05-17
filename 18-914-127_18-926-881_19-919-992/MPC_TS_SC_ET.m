%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC_ET
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC_ET(Q, R, N, H, h, S, v, S_t, v_t, params)
            
            % ADD STUFF HERE
            u = sdpvar(repmat(params.model.nu,1,N),repmat(1,1,N));
            x = sdpvar(repmat(params.model.nx,1,N+1),repmat(1,1,N+1));
            eta = sdpvar(repmat(length(S(:,1)),1,N),repmat(1,1,N));
            eta_t = sdpvar(repmat(length(h),1,1),repmat(1,1,1));
            
            constraints = [];
            objective = 0;
            
            [P,~,~] = idare(params.model.A,params.model.B,Q,R);
            
            Hx = params.constraints.StateMatrix;
            hx = params.constraints.StateRHS;
            Hu = params.constraints.InputMatrix;
            hu = params.constraints.InputRHS;
                       
            for k = 1:N
                objective = objective + x{k}'*Q*x{k} + u{k}'*R*u{k};
                objective = objective + eta{k}'*S*eta{k};
                objective = objective + v*norm(eta{k},1);
                constraints = [constraints, x{k+1} == params.model.A*x{k} + params.model.B*u{k}];
                constraints = [constraints, Hx * x{k} <= hx + eta{k}, Hu * u{k} <= hu, -eta{k} <= 0];
            end
            
            objective = objective + x{N+1}'*P*x{N+1} + eta_t'*S_t*eta_t + v_t*norm(eta_t,1);
            constraints = [constraints, H * x{N+1} <= h + eta_t, -eta_t <= 0];

            opts = sdpsettings('verbose', 1, 'solver', 'quadprog', 'quadprog.TolFun', 1e-8);
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

            ctrl_info = struct('ctrl_feas', feasible, 'objective', objective, 'solvetime', solvetime);
        end
    end
end
