%% LQGI_run_Jeremie.m  -  Jeremie's LQGI idle speed controller: linearization, design, robustness check, competition file
% Design idea (from the analysis of the cost J on the shared bench, README section 7):
%   - about 85-90 % of J is throttle activity caused by measurement noise, and moving the ignition is free in J
%   - so the ignition does the fast work and the throttle stays calm: large nominal retardation (-30 degCA, the range
%     suggested by the assistants is -40...-10) gives the ignition a large upward torque reserve; the k_max^2.5 penalty
%     (1.80 at -30) only multiplies the tracking term, which is the small part of J
%   - robustness: the weights were chosen by a search on a replica of the shared bench (random references and load,
%     recorded noise), requiring no stall on 3 random profiles AND on 9 parameter variants (inertia -15/+25 %,
%     alpha1 +/-20 %, Vm x0.6 and x1.5, eta0 -10 %, beta0 +30 %, gamma0 -10 %), and min|1+L| >= 0.5 at 70-200 rad/s
% The plant is linearized here at the design point (90 rad/s, -30 degCA), because LinModels_bench.mat only holds -7 and -15 degCA.
% Run time: about 2-4 minutes (5 steady-state searches); the linear models are cached in LinModels_Jeremie.mat.

pd = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});   pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models')];   addpath(strjoin(pd(isfolder(pd)), pathsep));
main
myDir = fileparts(mfilename('fullpath'));
ident = load('ident_fit_bench.mat').ident;          % parameters identified on our bench recordings

%% ---------------- tuning parameters ----------------
w_design = 90;      % rad/s   design speed
du_nom   = -30;     % degCA   nominal ignition retardation (ignition reserve; k_max^2.5 = 1.80)
P_design = 900;     % W       linearized constant-power load added to the design model (robustness at low speed)
r1   = 0.045;       % penalty on u_alpha (normalized)
r2   = 1;           % penalty on delta_u (normalized)
qw   = 52;          % output weight, Q = qw*C'*C
qobs = 2e-3;        % observer weight (larger = slower observer, less throttle noise)
K1   = 1;           % scaling of the integrator state of the extension
Ts   = 1e-3;        % s
saveFile = 'controller1_Jeremie.mat';
checkSpeeds = [70 100 140 200];                     % rad/s, robustness check (load off and 1.2 kW on)

%% ---------------- linear models (cached) ----------------
cacheFile = fullfile(myDir, 'LinModels_Jeremie.mat');
speeds = unique([w_design checkSpeeds]);
LM = struct([]);
if isfile(cacheFile), C0 = load(cacheFile); LM = C0.LM; end
for w = speeds
    if isempty(LM) || ~any(abs([LM.omega_e_nom]-w) < 0.5 & [LM.delta_u_nom] == du_nom)
        fprintf('linearizing at %g rad/s, %g degCA ...\n', w, du_nom);
        L1 = linAt(w, du_nom, ident, constants, limits, data);
        if isempty(LM), LM = L1; else, LM(end+1) = L1; end
        save(cacheFile, 'LM');
    end
end
pick = @(w) LM(find(abs([LM.omega_e_nom]-w) < 0.5 & [LM.delta_u_nom] == du_nom, 1));
Lm = pick(w_design);
fprintf('design point: %.1f rad/s, u_alpha_nom %.4f %%, p_m %.0f Pa, delta_u_nom %g degCA\n', Lm.omega_e_nom, Lm.u_alpha_nom, Lm.p_m_nom, du_nom);

%% ---------------- design model ----------------
As = Lm.A;  Bs = Lm.B;  Cs = Lm.C;  Ds = Lm.D;
iw = find(abs(Cs) > 0.5, 1);                         % speed state (y = normalized omega_e)
kl = @(P,w) P/(w^2*constants.eta_gen*ident.Theta_e); % linearized constant-power load, 1/s
As(iw,iw) = As(iw,iw) + kl(P_design, Lm.omega_e_nom);
[Tb,Ab] = balance(As);  Bb = Tb\Bs;  Cb = Cs*Tb;     % rescale the states (Pade companion forms)

%% ---------------- LQGI (manual p. 33-36) ----------------
Ae = 0;  Be = [1 0];  Ce = [K1; 0];  De = eye(2);
n  = size(Ab,1);
A  = [Ab, Bb*Ce; zeros(1,n), Ae];   B = [Bb*De; Be];   C = [Cb, Ds*Ce];
K  = lqr(A, B, qw*(C'*C), diag([r1 r2]));
Lg = lqr(A', C', B*B', qobs)';
nx = size(A,1);  Dc = zeros(2,1);
ISCS_Ac = [Ae, -Be*K; zeros(nx,1), A - B*K - Lg*C];
ISCS_Bc = [Be*Dc; -Lg];
ISCS_Cc = [Ce, -De*K];
ISCS_Dc = De*Dc;
sysc = ss(ISCS_Ac, ISCS_Bc, ISCS_Cc, ISCS_Dc);
sysd = c2d(sysc, Ts, 'tustin');
ISCS_Ad = sysd.A;  ISCS_Bd = sysd.B;  ISCS_Cd = sysd.C;  ISCS_Dd = sysd.D;
ISCS_Tu = diag([Lm.u_alpha_nom, du_nom]);
ISCS_Ty = Lm.omega_e_nom;
fprintf('slowest pole: state feedback %.2f, observer %.2f 1/s\n', min(abs(real(eig(A-B*K)))), min(abs(real(eig(A-Lg*C)))));

%% ---------------- robustness over the operating range ----------------
w = logspace(-2, 3, 3000);
fprintf('\n  omega   load | plant max Re | stable  min|1+L|  crossover\n');
allStable = true;  muMin = inf;
for wj = checkSpeeds
    Mj = pick(wj);
    for P = [0 1200]
        Aj = Mj.A;  Aj(iw,iw) = Aj(iw,iw) + kl(P, Mj.omega_e_nom);
        Sc = diag([ISCS_Tu(1,1)/Mj.u_alpha_nom, 1]) * (Mj.omega_e_nom/ISCS_Ty);   % controller units -> units of model j
        Lj = ss(Aj, Mj.B, Mj.C, Mj.D) * Sc * sysc;
        Lw = squeeze(freqresp(Lj, w));  mu = min(abs(1+Lw));  wc = w(find(abs(Lw) >= 1, 1, 'last'));  st = isstable(feedback(Lj,1));
        fprintf('  %5.0f  %5.0f | %+11.2f  | %4d    %7.3f   %6.2f rad/s\n', Mj.omega_e_nom, P, max(real(eig(Aj))), st, mu, wc);
        allStable = allStable && st;  muMin = min(muMin, mu);
    end
end
fprintf('all cases stable: %d, worst min|1+L| = %.3f (need >= 0.5)\n', allStable, muMin);

%% ---------------- competition file and closed-loop test ----------------
save(fullfile(myDir, saveFile), 'ISCS_Ad', 'ISCS_Bd', 'ISCS_Cd', 'ISCS_Dd', 'ISCS_Tu', 'ISCS_Ty');
fprintf('saved %s\n', saveFile);
test_controller(fullfile(myDir, saveFile));

%% ======================= local function =======================
function L = linAt(omega_target, delta_u_nom, ident, constants, limits, data) %#ok<INUSL>
% Linearization of Engine_Plant_Norm at (omega_target, delta_u_nom), same steps as LinNom_bench.m.
% The Simulink models read their parameters from the base workspace, so they are put there.
    theta_a_nom = mean(data.meas.T_a.signals.values);   p_a_nom = mean(data.meas.p_a.signals.values);
    theta_m_nom = mean(data.meas.T_m.signals.values);   p_e_nom = mean(data.meas.p_e.signals.values);
    V = struct('ident',ident,'theta_a_nom',theta_a_nom,'p_a_nom',p_a_nom,'theta_m_nom',theta_m_nom,'p_e_nom',p_e_nom,'delta_u_nom',delta_u_nom);
    f = fieldnames(V);  for i = 1:numel(f), assignin('base', f{i}, V.(f{i})); end
    model = 'Engine_Plant_SteadyState';  model_norm = 'Engine_Plant_Norm';
    load_system(model);  set_param(model,'StopTime','90');  load_system(model_norm);
    for name = {'omega_e','mdot_beta','p_m'}
        ln = find_system(model,'FindAll','on','Type','line','Name',name{1});
        set_param(get_param(ln(1),'SrcPortHandle'),'DataLogging','on');
    end
    u_alpha_nom = fzero(@(u) ssRun(model,u) - omega_target, [0.3 8], optimset('TolX',1e-5));
    [omega_e_nom, mdot_beta_nom, p_m_nom, drift] = ssRun(model, u_alpha_nom);
    Q_nom = mdot_beta_nom/omega_e_nom;
    tau_IPS = 2*pi/omega_e_nom;  tau_seg = 2*pi/(5*omega_e_nom);
    [num1,den1] = pade(tau_IPS,4);  [A1,B1,C1,D1] = tf2ss(num1,den1);
    [num2,den2] = pade(tau_seg,4);  [A2,B2,C2,D2] = tf2ss(num2,den2);
    x0_1 = -A1\B1;   x0_2 = -A2\B2;
    V = struct('u_alpha_nom',u_alpha_nom,'omega_e_nom',omega_e_nom,'p_m_nom',p_m_nom,'Q_nom',Q_nom, ...
               'A1',A1,'B1',B1,'C1',C1,'D1',D1,'x0_1',x0_1,'A2',A2,'B2',B2,'C2',C2,'D2',D2,'x0_2',x0_2);
    f = fieldnames(V);  for i = 1:numel(f), assignin('base', f{i}, V.(f{i})); end
    [~,x0] = feval(model_norm,[],[],[],'sizes');
    [A,B,C,D] = linmod(model_norm, x0, [1;1]);
    fprintf('   u_alpha_nom %.4f %%, drift over last 5 s %.3f rad/s, max Re(eig) %.3f\n', u_alpha_nom, drift, max(real(eig(A))));
    L = struct('A',A,'B',B,'C',C,'D',D,'omega_e_nom',omega_e_nom,'u_alpha_nom',u_alpha_nom,'delta_u_nom',delta_u_nom, ...
               'p_m_nom',p_m_nom,'Q_nom',Q_nom,'drift',drift);
end

function [w, mb, pm, drift] = ssRun(model, u)
    in  = Simulink.SimulationInput(model);
    in  = in.setVariable('u_alpha_nom', u);
    out = sim(in);
    s   = out.logsout.get('omega_e').Values;   k = s.Time >= s.Time(end)-10;     % average over the last 10 s
    w   = mean(s.Data(k));                                                     % (the low-speed model keeps a small oscillation)
    m1  = out.logsout.get('mdot_beta').Values;  mb = mean(m1.Data(m1.Time >= m1.Time(end)-10));
    p1  = out.logsout.get('p_m').Values;        pm = mean(p1.Data(p1.Time >= p1.Time(end)-10));
    drift = mean(s.Data(s.Time >= s.Time(end)-5)) - mean(s.Data(s.Time >= s.Time(end)-10 & s.Time < s.Time(end)-5));
end
