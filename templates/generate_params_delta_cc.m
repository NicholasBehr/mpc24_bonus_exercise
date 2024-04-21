%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params_delta = generate_params_delta_cc(params)
    
params_delta = params;

params_delta.model = rmfield(params_delta.model, 'C');
params_delta.model = rmfield(params_delta.model, 'Cd');
params_delta.model = rmfield(params_delta.model, 'C_ref');
params_delta.model = rmfield(params_delta.model, 'D');
params_delta.model = rmfield(params_delta.model, 'Bd');

% CONTINUE BELOW THIS LINE

end