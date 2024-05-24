
q = [1 1 0] * 1;
r = [2.4 0.5] * 10^(-5);

[params] = generate_params_cc();
[params] = generate_params_delta_cc(params);

Q = diag(q);
R = diag(r);
x0 = params.exercise.InitialConditionA;
N = 30;

ctrl = MPC(Q,R,N,params);

% ctrl = LQR(Q,R,params);

[x,u,ctrl_info] = simulate(x0, ctrl, params);

[x, u] = traj_delta2abs(x, u, params.exercise.x_s, params.exercise.u_s);

[fig_time,axes_time] = plot_trajectory_cc(x,u,ctrl_info,params);
