%% ParID_bench.m  -  identification on OUR bench measurements (2 Nov 2026)
% Same method as ParID.m, with three differences:
%   - fminsearch instead of fmincon (no Optimization Toolbox needed), bounds by projection
%   - dynamic fit on the FIRST 60 s of studentId_0003, validation on the LAST 60 s (never fitted)
%   - result saved in ident_fit_bench.mat (ident_fit.mat of the repository is left untouched)
% Static part (alpha, gamma): studentId_0002 (slow throttle staircase, no load).
% Before running: main.m must contain  data_name = 'studentId_0003';

%% 0. Load parameters and studentId_0003
addpath(strjoin(fullfile(fileparts(fileparts(mfilename('fullpath'))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'}), pathsep));   % milestone folders
main;
Tfit = 60;                               % s  fit window [0, Tfit], validation window (Tfit, end]

%% 1. Throttle: alpha0, alpha1 (linear least squares)
q  = load(fullfile(fileparts(which('main')),'..','..','ISCSData','studentId_0002.mat')).meas;
ua = q.u_alpha.signals.values(:);      md = q.m_dot_alpha.signals.values(:);
Ta = q.T_a.signals.values(:);          pa = q.p_a.signals.values(:);
pm = q.p_m.signals.values(:);          pe = q.p_e.signals.values(:);
Tm = q.T_m.signals.values(:);          we = q.omega_e.signals.values(:);
N  = numel(ua);
c  = constants;
fprintf('p_m/p_a in studentId_0002: %.2f to %.2f (must stay below 0.5 for the choked-flow formula)\n', min(pm./pa), max(pm./pa));

y  = md.*sqrt(2*c.R*Ta)./pa;             % choked flow: A_alpha = mdot*sqrt(2*R*theta_a)/p_a
M  = [ones(N,1) ua];
xa = M\y;
ident.alpha0 = xa(1);  ident.alpha1 = xa(2);
figure(1); clf; plot(ua, y, '.', ua, M*xa, '-'); xlabel('u_\alpha [%]'); ylabel('A_\alpha [m^2]'); grid on

%% 2. Volumetric efficiency: gamma0, gamma1 (linear least squares, 70-200 rad/s only)
lam_lp = (c.Vc+c.Vd)/c.Vd - c.Vc/c.Vd*(pe./pm).^(1/c.kappa);
lam_lw = 4*pi*c.R/c.Vd .* Tm.*md./(pm.*we.*lam_lp) * (1+1/(c.lambda*c.sigma0));
k  = (we>=70 & we<=200) & ((1:N)'>1000) & ((1:N)'<N-1000);
xg = [ones(nnz(k),1) we(k)] \ lam_lw(k);
ident.gamma0 = xg(1);  ident.gamma1 = xg(2);
figure(2); clf; plot(we, lam_lw, '.', we, xg(1)+xg(2)*we, '-'); xlabel('\omega_e [rad/s]'); ylabel('\lambda_{l\omega}'); grid on

% q_init depends on gamma -> recompute with the new values
pe1 = data.meas.p_e.signals.values(1);  Tm1 = data.meas.T_m.signals.values(1);
lp1 = (c.Vc+c.Vd)/c.Vd - c.Vc/c.Vd*(pe1/init.p_m0)^(1/c.kappa);
init.q_init = init.p_m0*c.Vd/(4*pi*c.R*Tm1)*lp1*(ident.gamma0+ident.gamma1*init.omega_e0)/(1+1/(c.lambda*c.sigma0));
fprintf('alpha0 = %.4g  alpha1 = %.4g  gamma0 = %.4f  gamma1 = %.4g\n', ident.alpha0, ident.alpha1, ident.gamma0, ident.gamma1);

%% 3. Manifold volume Vm (measured speed fed to the model, fit window only)
mdl = 'Engine_Plant';  load_system(mdl);
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('SimulationMode','accelerator', 'StopTime',num2str(Tfit), 'EnablePacing','off');
in = in.setVariable('init', init);
sc = simcfg;  sc.omega_meas = 1;
in = in.setVariable('simcfg', sc);
kf = data.t <= Tfit;                     % samples of the fit window

ident.Vm = fminsearch(@(Vm) costVmW(Vm, in, ident, data, kf), 7e-3, ...
                      optimset('Display','iter','TolX',1e-5,'MaxFunEvals',60));
fprintf('Vm = %.4g m^3\n', ident.Vm);

%% 4. eta0, eta1, beta0, Theta_e (simulated speed, fit window only)
sc.omega_meas = 0;
in = in.setVariable('simcfg', sc);
p0 = [0.3, -3e-4, 7, 0.2];                % scaling = manual initial guesses
lb = [0.05, -2e-3,  0, 0.05] ./ abs(p0);
ub = [0.6,   2e-3, 30, 1   ] ./ abs(p0);
x0 = p0 ./ abs(p0);                       % start from the manual guesses: [1 -1 1 1]
cl = @(x) min(max(x, lb), ub);            % projection on the bounds
J  = @(x) costTorqueW(cl(x), in, ident, data, abs(p0), kf) + 100*sum((x-cl(x)).^2);
t0 = tic;
stopfun = @(x,ov,state) toc(t0) > 20*60;  % hard stop after 20 minutes
opts = optimset('Display','iter','MaxFunEvals',300,'MaxIter',150,'TolX',1e-3,'TolFun',1e-3,'OutputFcn',stopfun);
xT = cl(fminsearch(J, x0, opts));
pT = xT.*abs(p0);
ident.eta0 = pT(1); ident.eta1 = pT(2); ident.beta0 = pT(3); ident.Theta_e = pT(4);
save(fullfile(fileparts(which('ParID_bench')),'ident_fit_bench.mat'),'ident')
disp(ident)

%% 5. Validation: whole recording with the fitted parameters, error on each half
inV = in.setModelParameter('StopTime', num2str(simcfg.stop_time)).setVariable('ident', ident);
out = sim(inV);
sw = out.logsout.get('omega_e').Values;  sp = out.logsout.get('p_m').Values;
w_sim   = interp1(sw.Time, sw.Data(:), data.t);
pm_sim  = interp1(sp.Time, sp.Data(:), data.t);
w_meas  = data.meas.omega_e.signals.values(:);
pm_meas = data.meas.p_m.signals.values(:);
kv = ~kf & ~isnan(w_sim);
fprintf('\n%-28s %14s %12s\n', 'studentId_0003', 'RMSE omega_e', 'RMSE p_m');
fprintf('%-28s %8.2f rad/s %9.0f Pa\n', sprintf('fit window  0-%g s', Tfit), rms(w_sim(kf)-w_meas(kf)), rms(pm_sim(kf)-pm_meas(kf)));
fprintf('%-28s %8.2f rad/s %9.0f Pa\n', sprintf('validation  %g-%g s', Tfit, data.t(end)), rms(w_sim(kv)-w_meas(kv)), rms(pm_sim(kv)-pm_meas(kv)));
fprintf('lowest speed: measured %.1f rad/s, simulated %.1f rad/s\n', min(w_meas), min(w_sim));
figure(3); clf;
subplot(2,1,1); plot(data.t, w_meas, 'k', data.t, w_sim, 'r'); xline(Tfit,'--'); grid on; ylabel('\omega_e [rad/s]'); legend('measured','model','Location','best'); title('studentId\_0003: fit (left of dashed line) and validation (right)');
subplot(2,1,2); plot(data.t, pm_meas, 'k', data.t, pm_sim, 'r'); xline(Tfit,'--'); grid on; ylabel('p_m [Pa]'); xlabel('t [s]');

%% Local functions: same costs as costVm.m and costTorque.m, restricted to the samples kf
function e = costVmW(Vm, in, ident, data, kf)
    ident.Vm = Vm;
    out = sim(in.setVariable('ident', ident));
    s   = out.logsout.get('p_m').Values;
    pm_sim  = interp1(s.Time, s.Data(:), data.t(kf));
    pm_meas = data.meas.p_m.signals.values(:);
    e = sum((pm_sim - pm_meas(kf)).^2, 'omitnan');
end

function e = costTorqueW(x, in, ident, data, p0, kf)
    p = x.*p0;
    ident.eta0 = p(1); ident.eta1 = p(2); ident.beta0 = p(3); ident.Theta_e = p(4);
    out = sim(in.setVariable('ident', ident));
    sw = out.logsout.get('omega_e').Values;  sp = out.logsout.get('p_m').Values;
    w_sim  = interp1(sw.Time, sw.Data(:), data.t(kf));
    pm_sim = interp1(sp.Time, sp.Data(:), data.t(kf));
    w_meas  = data.meas.omega_e.signals.values(:);   w_meas  = w_meas(kf);
    pm_meas = data.meas.p_m.signals.values(:);       pm_meas = pm_meas(kf);
    e = sum((w_sim-w_meas).^2, 'omitnan')/var(w_meas) + sum((pm_sim-pm_meas).^2, 'omitnan')/var(pm_meas);
end
