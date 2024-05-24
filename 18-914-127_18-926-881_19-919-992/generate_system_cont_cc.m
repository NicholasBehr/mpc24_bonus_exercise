%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc, Bdc] = generate_system_cont_cc(params)

    Ac = [(-1/params.model.m1)*(params.model.a12 + params.model.a1o) , (1/params.model.m1)*params.model.a12 , 0 ;
          (1/params.model.m2)*params.model.a12 , (-1/params.model.m2)*(params.model.a12 + params.model.a23 + params.model.a2o) , (1/params.model.m2)*params.model.a23 ;
          0 , (1/params.model.m3)*params.model.a23 , (-1/params.model.m3)*(params.model.a23 + params.model.a3o)];

    Bc = [1/params.model.m1 , 0 ;
          0 , 1/params.model.m2 ;
          0 , 0 ];

    Bdc = [1/params.model.m1 , 0 , 0 ;
           0 , 1/params.model.m2 , 0 ;
           0 , 0 , 1/params.model.m3 ];

end