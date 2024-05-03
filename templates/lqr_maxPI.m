%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H, h] = lqr_maxPI(Q, R, params)

obj = LQR(Q,R,params);
K = obj.K;
 
A = params.model.A;
B = params.model.B;
 
Hx = params.constraints.StateMatrix;
hx = params.constraints.StateRHS;
Hu = params.constraints.InputMatrix;
hu = params.constraints.InputRHS;
 
H_aug = [Hx;Hu*K];
h_aug = [hx; hu];
 
system = LTISystem('A', A+B*K);
Px = Polyhedron('H', [H_aug h_aug]);

% Px.isBounded()
% Px.isFullDim()
% Px.plot()
 
system.x.with('setConstraint');
system.x.setConstraint = Px;
InvSet = system.invariantSet();

H = InvSet.A;
h = InvSet.b;
 
% P = Polyhedron('H', [H h]);
% P.isBounded()
% P.isEmptySet()
% P.plot()

end

