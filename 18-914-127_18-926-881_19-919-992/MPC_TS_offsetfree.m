%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_offsetfree
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_offsetfree(Q,R,N,H,h,params)

            % ADD STUFF HERE
            u = sdpvar(repmat(params.model.nu,1,N),repmat(1,1,N));
            x = sdpvar(repmat(params.model.nx,1,N+1),repmat(1,1,N+1));
            X_S = sdpvar(params.model.nx,1);
            U_S = sdpvar(params.model.nu,1);
            D = sdpvar(params.model.nd,1);
            
            constraints = [];
            objective = 0;
            
            [P,~,~] = idare(params.model.A,params.model.B,Q,R);
            
            Hx = params.constraints.StateMatrix;
            hx = params.constraints.StateRHS;
            Hu = params.constraints.InputMatrix;
            hu = params.constraints.InputRHS;
                       
            for k = 1:N
                objective = objective + (x{k}-X_S)'*Q*(x{k}-X_S) + (u{k}-U_S)'*R*(u{k}-U_S);
                constraints = [constraints, x{k+1} == params.model.A*x{k} + params.model.B*u{k} + params.model.Bd*D];
                constraints = [constraints, Hx * x{k} <= hx, Hu * u{k} <= hu];
            end
            objective = objective + (x{N+1}-X_S)'*P*(x{N+1}-X_S);
            constraints = [constraints, H * (x{N+1}-X_S) <= h];

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,{x{1} X_S U_S D},{u{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x,d,x_s,u_s)
            tic
            [optimizer_out,errorcode] = obj.yalmip_optimizer({x x_s u_s d});
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