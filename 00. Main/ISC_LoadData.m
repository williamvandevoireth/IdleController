% ISC_LoadData.m
% Builds the time series read by the From Workspace blocks of Engine_Plant, plus the input matrix of the two inports.
% Run AFTER ISC_Parameters. Options (set before running): data_file, dataset.
if ~exist('data_file','var'), data_file = 'DummyMeasurementData.mat'; end   % replace by the real bench file
if ~exist('dataset','var'),   dataset   = 'DataIdentification';       end   % or 'DataValidation'
S = load(data_file, dataset);
D = S.(dataset);
theta_a_ts = timeseries(D.theta_a, D.t);   % K    ambient temperature
p_a_ts     = timeseries(D.p_a,     D.t);   % Pa   ambient pressure
theta_m_ts = timeseries(D.theta_m, D.t);   % K    intake manifold temperature
p_e_ts     = timeseries(D.p_e,     D.t);   % Pa   exhaust pressure
u_l_ts     = timeseries(D.u_l,     D.t);   % 1/0  load on/off
P_l_ts     = timeseries(D.P_l,     D.t);   % W    load power
ext_in = [D.t, D.delta_u_zeta, D.u_alpha]; % inports: port 1 = delta_u [degCA], port 2 = u_alpha [%]
clear S