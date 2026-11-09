%% Validation.m  (manual p.29: compare the model with data NOT used for the identification)
% Run after ParID.m (needs ident_fit.mat). main.m does not need to be edited.
% Simulates dynamic_0002 (fit data) and dynamic_0003 (validation data) with
%   - the manual's initial guesses (before identification)
%   - the identified parameters (ident_fit.mat)
% and plots omega_e and p_m against the measurement, with the RMSE of each.

main;                                       % constants, limits, simcfg, ident = initial guesses
if ~isfile('ident_fit.mat'), error('ident_fit.mat not found: run ParID.m (sections 0-4) first.'); end
P.initial = ident;                          % manual initial guesses
P.fitted  = load('ident_fit.mat').ident;    % result of ParID.m
sets  = fieldnames(P);
files = {'studentId_0002','studentId_0003'};    % 0002 = used for fitting, 0003 = validation (never fitted)
mdl = 'Engine_Plant';  load_system(mdl);

fprintf('\n%-14s %-9s %16s %14s\n','data','params','RMSE omega_e','RMSE p_m');
for f = 1:numel(files)
    meas = load(fullfile('..','ISCSData',[files{f} '.mat'])).meas;
    t    = meas.time(:);
    w_meas  = meas.omega_e.signals.values(:);
    pm_meas = meas.p_m.signals.values(:);

    figure(10+f); clf;
    subplot(2,1,1); plot(t, w_meas, 'k'); hold on; grid on; ylabel('\omega_e [rad/s]');
    role = {'fit data','validation data'};
    title(sprintf('%s (%s)', strrep(files{f},'_','\_'), role{f}));
    subplot(2,1,2); plot(t, pm_meas, 'k'); hold on; grid on; ylabel('p_m [Pa]'); xlabel('t [s]');

    for s = 1:numel(sets)
        id = P.(sets{s});
        [dat, ini] = runData(meas, constants, id);
        sc = simcfg;  sc.stop_time = t(end);  sc.omega_meas = 0;    % full model, simulated speed

        in = Simulink.SimulationInput(mdl);
        in = in.setModelParameter('SimulationMode','accelerator', 'StopTime',num2str(t(end)), 'EnablePacing','off');
        in = in.setVariable('data',dat).setVariable('init',ini).setVariable('ident',id).setVariable('simcfg',sc);
        in = in.setExternalInput(dat.ext_in);
        out = sim(in);

        sw = out.logsout.get('omega_e').Values;  sp = out.logsout.get('p_m').Values;
        w  = interp1(sw.Time, sw.Data(:), t);
        pm = interp1(sp.Time, sp.Data(:), t);
        fprintf('%-14s %-9s %12.2f rad/s %11.0f Pa\n', files{f}, sets{s}, rms(w-w_meas), rms(pm-pm_meas));

        subplot(2,1,1); plot(t, w);  subplot(2,1,2); plot(t, pm);
    end
    subplot(2,1,1); legend(['measured'; sets], 'Location','best');
end

%% Local function: data struct and initial conditions for one recording (same as main.m)
function [data, init] = runData(meas, c, id)
    data.meas   = meas;
    data.t      = meas.time(:);
    data.ext_in = [data.t, meas.u_alpha.signals.values(:), meas.du_ign.signals.values(:)];   % port 1 u_alpha, port 2 delta_u
    init.omega_e0 = meas.omega_e.signals.values(1);
    init.p_m0     = meas.p_m.signals.values(1);
    init.du_init  = meas.du_ign.signals.values(1);
    pe1 = meas.p_e.signals.values(1);  Tm1 = meas.T_m.signals.values(1);
    lp1 = (c.Vc+c.Vd)/c.Vd - c.Vc/c.Vd*(pe1/init.p_m0)^(1/c.kappa);
    init.q_init = init.p_m0*c.Vd/(4*pi*c.R*Tm1)*lp1*(id.gamma0+id.gamma1*init.omega_e0)/(1+1/(c.lambda*c.sigma0));
end
