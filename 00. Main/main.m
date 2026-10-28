% main.m
% Sets up the workspace for Engine_Plant: parameters and measurement data, grouped in structs.
%
% different structs:
%   constants  known physical constants and given coefficients   (R, Vd, Vc, sigma0, kappa, lambda, Hl, k_zeta, eta_gen)
%   ident      parameters to identify, currently initial guesses  (alpha0, alpha1, Vm, gamma0, gamma1, eta0, eta1, beta0, Theta_e)
%   init       initial conditions of the dynamic blocks           (omega_e0, p_m0, du_init, q_init)
%   limits     engine speed limits and longest delay              (w_min, w_max, tau_max)
%   simcfg     simulation settings                                (Ts, stop_time)
%   data       measurement data                                   (meas, t, ext_in, ts.*)
clear

data_name = 'dynamic_0002';   % file name without .mat

main_dir = fileparts(mfilename('fullpath'));
cd(main_dir)
ISC_Parameters                % plain variables, grouped below

load(fullfile(main_dir, '..', 'ISCSData', [data_name '.mat']), 'meas');

%% --- Parameters (same grouping as ISC_Parameters.m)
constants = struct('R',R, 'Vd',Vd, 'Vc',Vc, 'sigma0',sigma0, 'kappa',kappa, 'lambda',lambda, ...
                   'Hl',Hl, 'k_zeta',k_zeta, 'eta_gen',eta_gen);
ident     = struct('alpha0',alpha0, 'alpha1',alpha1, 'Vm',Vm, 'gamma0',gamma0, 'gamma1',gamma1, ...
                   'eta0',eta0, 'eta1',eta1, 'beta0',beta0, 'Theta_e',Theta_e);
init      = struct('omega_e0',omega_e0, 'p_m0',p_m0, 'du_init',du_init, 'q_init',q_init);
limits    = struct('w_min',w_min, 'w_max',w_max, 'tau_max',tau_max);

%% --- Measurement data
data.meas = meas;                   % read by the From Workspace blocks (data.meas.T_a, ...)
data.t    = meas.time(:);           % s    common time vector, 1 kHz (matches Ts)

% time series of the measured signals (not read by any block, kept for plotting/identification)
data.ts.theta_a = timeseries(meas.T_a.signals.values(:), data.t);   % K    ambient temperature
data.ts.p_a     = timeseries(meas.p_a.signals.values(:), data.t);   % Pa   ambient pressure
data.ts.theta_m = timeseries(meas.T_m.signals.values(:), data.t);   % K    intake manifold temperature
data.ts.p_e     = timeseries(meas.p_e.signals.values(:), data.t);   % Pa   exhaust pressure
data.ts.u_l     = timeseries(meas.u_l.signals.values(:), data.t);   % 1/0  load on/off
data.ts.P_l     = timeseries(meas.P_l.signals.values(:), data.t);   % W    load power

% inports: port 1 = delta_u [degCA] (ignition), port 2 = u_alpha [%] (throttle)
data.ext_in = [data.t, meas.du_ign.signals.values(:), meas.u_alpha.signals.values(:)];

simcfg = struct('Ts',Ts, 'stop_time',data.t(end));   % stop_time: simulate over the whole recording

%% --- Remove the plain variables, only the structs stay
clear R Vd Vc sigma0 kappa lambda Hl k_zeta eta_gen
clear alpha0 alpha1 Vm gamma0 gamma1 eta0 eta1 beta0 Theta_e
clear omega_e0 p_m0 du_init q_init pe_nom Tm_nom lam_lp0
clear w_min w_max tau_max Ts
clear meas data_name main_dir
