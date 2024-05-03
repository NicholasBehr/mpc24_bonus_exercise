%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC(Q,R,N,params)
            
            % ADD STUFF HERE
            U = sdpvar(repmat(params.model.nu,1,N),repmat(1,1,N));
            X0 = sdpvar(params.model.nx,1);
            
            constraints = [];
            objective = 0;
            x = X0;
            P = zeros(params.model.nx, params.model.nx);

            for k = 1:N
                x = params.model.A*x + params.model.B*U{k};
                objective = objective + norm(Q*x,1) + norm(R*U{k},1);
                constraints = [constraints, -1 <= U{k}<= 1, -5<=x<=5];
            end
            objective = objective + norm(P*x,1);

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
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