%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ad, Bd, Bd_d] = discretize_system_dist(Ac, Bc, Bd_c, params)
disp(Ac)
sys_c = ss(Ac,[Bc Bd_c],zeros(size(Ac)),0);
sys_d = c2d(sys_c, params.model.TimeStep);

Ad = sys_d.A;
Bd = sys_d.B(:, 1:size(Bc, 2));
Bd_d = sys_d.B(:, size(Bc, 2)+1:end);



end