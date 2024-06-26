%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints_cc(params)
        
    H_x = [1 , 0 , 0 ;
           -1, 0 , 0 ;
           0 , 1 , 0 ;
           0 , -1, 0 ;
           0 , 0 , 1 ;
           0 , 0 ,-1 ];
    h_x = [params.constraints.T1Max ; -params.constraints.T1Min ; params.constraints.T2Max ; ...
        -params.constraints.T2Min ; params.constraints.T3Max ; -params.constraints.T3Min];

    H_u = [1 , 0 ;
           -1, 0 ;
           0 , 1 ;
           0 ,-1 ];
    h_u = [params.constraints.P1Max ; -params.constraints.P1Min ; ...
           params.constraints.P2Max ; -params.constraints.P2Min ];

    
end

