%% 0. Load parameters and dynamic_0002 (check data_name = 'dynamic_0002' in main.m)
main;

%% 1. Throttle: alpha0, alpha1
q  = load('../ISCSData/quasistatic_0001.mat').meas;   % quasistatic measurement, all N samples
ua = q.u_alpha.signals.values(:);      md = q.m_dot_alpha.signals.values(:);
Ta = q.T_a.signals.values(:);          pa = q.p_a.signals.values(:);
pm = q.p_m.signals.values(:);          pe = q.p_e.signals.values(:);
Tm = q.T_m.signals.values(:);          we = q.omega_e.signals.values(:);
N  = numel(ua);
c  = constants;

y  = md.*sqrt(2*c.R*Ta)./pa;             % Eq. 2.2/2.3: y_i = mdot_alpha,i*sqrt(2*R*theta_a,i)/p_a,i
M  = [ones(N,1) ua];                     % Eq. 2.3: M = [1 u_alpha,i]
xa = M\y;                                % linear LS solution x = [alpha0; alpha1]
ident.alpha0 = xa(1);  ident.alpha1 = xa(2);
figure(1); clf; plot(ua, y, '.', ua, M*xa, '-'); xlabel('u_\alpha [%]'); ylabel('A_\alpha [m^2]'); grid on

%% 2. Volumetric efficiency: gamma0, gamma1
lam_lp = (c.Vc+c.Vd)/c.Vd - c.Vc/c.Vd*(pe./pm).^(1/c.kappa);
lam_lw = 4*pi*c.R/c.Vd .* Tm.*md./(pm.*we.*lam_lp) * (1+1/(c.lambda*c.sigma0));   % Eq. 2.4
xg = [ones(N,1) we]\lam_lw;              % Eq. 2.5: x = [gamma0; gamma1]
ident.gamma0 = xg(1);  ident.gamma1 = xg(2);
figure(2); clf; plot(we, lam_lw, '.', we, xg(1)+xg(2)*we, '-'); xlabel('\omega_e [rad/s]'); ylabel('\lambda_{l\omega}'); grid on

% q_init depends on gamma -> recompute with the new values
pe1 = data.meas.p_e.signals.values(1);  Tm1 = data.meas.T_m.signals.values(1);
lp1 = (c.Vc+c.Vd)/c.Vd - c.Vc/c.Vd*(pe1/init.p_m0)^(1/c.kappa);
init.q_init = init.p_m0*c.Vd/(4*pi*c.R*Tm1)*lp1*(ident.gamma0+ident.gamma1*init.omega_e0)/(1+1/(c.lambda*c.sigma0));

%% 3. Manifold volume Vm
mdl = 'Engine_Plant';  load_system(mdl);
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('SimulationMode','accelerator', 'StopTime',num2str(simcfg.stop_time), 'EnablePacing','off');
in = in.setVariable('init', init);
sc = simcfg;  sc.omega_meas = 1;          % use MEASURED speed
in = in.setVariable('simcfg', sc);

ident.Vm = fminsearch(@(Vm) costVm(Vm, in, ident, data), 7e-3, ...   % initial guess 7e-3 m^3 (manual p.19)
                      optimset('Display','iter','TolX',1e-6))

%% 4. eta0, eta1, beta0, Theta_e
sc.omega_meas = 0;                        % full model, simulated speed
in = in.setVariable('simcfg', sc);
p0 = [0.3, -3e-4, 7, 0.2];                % manual initial guesses
lb = [0.05, -2e-3,  0, 0.05] ./ abs(p0);  % lower bounds (scaled)
ub = [0.6,   2e-3, 30, 1   ] ./ abs(p0);  % upper bounds (scaled)
x0 = p0 ./ abs(p0);                       % = [1 -1 1 1]
opts = optimoptions('fmincon','Display','iter','FiniteDifferenceStepSize',1e-2);
xT = fmincon(@(x) costTorque(x, in, ident, data, abs(p0)), x0, [],[],[],[], lb, ub, [], opts);
pT = xT.*abs(p0);
ident.eta0 = pT(1); ident.eta1 = pT(2); ident.beta0 = pT(3); ident.Theta_e = pT(4);
save ident_fit.mat ident                  % keep the result, main.m would overwrite it
disp(ident)