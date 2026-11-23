function R = robust_test(files, opt)
%ROBUST_TEST  Robust closed-loop evaluation of competition controllers on the nonlinear engine model.
%   Extends the shared bench (M4 Competition/test_controller.m) with what it does not test:
%     1. performance on 10 random profiles (instead of 1-3), with the recorded bench noise scaled by 1.5
%        (noise measured in studentId_0003 away from load switching: 0.31-0.42 rad/s, vs 0.21 rad/s in the shared bench)
%        and taken at a different offset for each profile (the shared bench always uses the same noise trace)
%     2. stress cases, each on 3 profiles: throttle actuator lag (20 ms delay + 50 ms first order, seen in our
%        mass-flow data), noise x2, and 9 parameter variants (inertia -15/+25 %, alpha1 +/-20 %, Vm x0.6 and x1.5,
%        eta0 -10 %, beta0 +30 %, gamma0 -10 %)
%     3. extra criteria: speed overshoot when the load switches off, steady-state error
%   R = robust_test({'controller1_Jeremie.mat','controller1_William.mat'})        all tests, comparison table
%   R = robust_test('controller1_Jeremie.mat', quick=true)                        3 profiles only (faster)
%   Options: seeds (1:10), nscale (1.5), du_min (-30), w_low (90), quick (false), P_on (1200)
%   Runs on a private copy of the shared bench (Bench_ISCS_Robust.slx in M4 Competition/models, created on first use; the only
%   change is the controller initial state, which accepts controllers with extra states). Shared files are not modified.
%   Run time: about 40 simulations per controller (2-4 min each controller).
arguments
    files
    opt.seeds  (1,:) double = 1:10
    opt.nscale (1,1) double = 1.5
    opt.du_min (1,1) double = -30
    opt.w_low  (1,1) double = 90
    opt.P_on   (1,1) double = 1200
    opt.quick  (1,1) logical = false
end
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));   m3 = fullfile(root, 'M3 Controller');   modelDir = fullfile(root, 'M4 Competition', 'models');
pd = fullfile(fileparts(fileparts(here)), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});
pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models'), fullfile(pd,'_Models'), fullfile(pd,'_Testing')];   addpath(strjoin(pd(isfolder(pd)), pathsep));   addpath(here);
if opt.quick, opt.seeds = 1:3; end
files = cellstr(files);

% private bench model
mdl = 'Bench_ISCS_Robust';
if ~isfile(fullfile(modelDir, [mdl '.slx']))
    load_system('Bench_ISCS');   save_system('Bench_ISCS', fullfile(modelDir, [mdl '.slx']));
    set_param([mdl '/ISCS'], 'InitialCondition', 'zeros(size(ISCS_Ad,1),1)');   save_system(mdl);
    fprintf('created %s.slx (copy of Bench_ISCS.slx)\n', mdl);
end
load_system(mdl);

m  = getMain();
S  = load('LinModels_bench.mat');   Mb = S.M;   ident0 = S.ident;
Lm = load('LinModel_bench.mat');
q  = load(fullfile(fileparts(which('main')), '..', '..', 'ISCSData', 'studentId_0002.mat')).meas;
wq = q.omega_e.signals.values(:);   N = numel(m.data.meas.time);   noise0 = wq(1:N) - movmean(wq(1:N), 200);

P = {'Theta_e',0.85,'Theta-15%'; 'Theta_e',1.25,'Theta+25%'; 'alpha1',1.2,'alpha1+20%'; 'alpha1',0.8,'alpha1-20%'; ...
     'Vm',0.6,'Vm x0.6'; 'Vm',1.5,'Vm x1.5'; 'eta0',0.9,'eta0-10%'; 'beta0',1.3,'beta0+30%'; 'gamma0',0.9,'gamma0-10%'};
sh = @(s) mod(s*7919*13, N);                                           % noise offset of profile s
for f = 1:numel(files)
    file = findFile(files{f}, m3);   c = load(file);   [~, nm] = fileparts(file);   clear r
    fprintf('\n=== %s ===\n', nm);
    init = initFor(c, m, Mb, m3);
    base = struct('m',m,'c',c,'ident',ident0,'init',init,'Lm',Lm,'noise0',noise0,'opt',opt,'mdl',mdl);
    % 1. performance
    for i = 1:numel(opt.seeds), r(i) = runOne(base, opt.seeds(i), opt.nscale, sh(opt.seeds(i)), ident0, 0, 0); end %#ok<AGROW>
    R(f).file = nm;  R(f).J = mean([r.J]);  R(f).J_track = mean([r.J_track]);  R(f).J_act = mean([r.J_act]);
    R(f).J_std = std([r.J]);  R(f).omega_min = min([r.omega_min]);  R(f).overshoot_loadoff = max([r.ov]);  R(f).ss_error = max([r.sse]);
    fprintf('  performance (%d profiles, noise x%.1f): J = %.0f +/- %.0f (tracking %.0f + throttle %.0f), min speed %.1f, load-off overshoot %.1f rad/s, steady error %.2f rad/s\n', ...
            numel(opt.seeds), opt.nscale, R(f).J, R(f).J_std, R(f).J_track, R(f).J_act, R(f).omega_min, R(f).overshoot_loadoff, R(f).ss_error);
    % 2. stress cases (3 profiles each)
    st = {};
    st(end+1,:) = {'throttle lag 20+50 ms', minSpeed(base, opt.nscale, sh, ident0, 20, 0.05)};
    st(end+1,:) = {'noise x2',              minSpeed(base, 2,           sh, ident0, 0, 0)};
    for i = 1:size(P,1)
        id = ident0;   id.(P{i,1}) = id.(P{i,1})*P{i,2};
        st(end+1,:) = {P{i,3}, minSpeed(base, opt.nscale, sh, id, 0, 0)}; %#ok<AGROW>
    end
    R(f).stress = cell2struct(st, {'name','omega_min'}, 2);
    w = [st{:,2}];   [R(f).worst_stress, iw] = min(w);   R(f).worst_case = st{iw,1};   R(f).stalls = sum(w < 50);
    for i = 1:size(st,1), fprintf('    %-22s min speed %6.1f rad/s%s\n', st{i,1}, st{i,2}, ifelse(st{i,2} < 50, '   STALL', '')); end
end
fprintf('\n%-30s %7s %6s %9s %9s %9s %11s %9s %7s\n', 'controller', 'J mean', '+/-', 'min w', 'overshoot', 'ss error', 'worst case', 'min w', 'stalls');
for f = 1:numel(R)
    fprintf('%-30s %7.0f %6.0f %9.1f %9.1f %9.2f %11s %9.1f %7d\n', R(f).file, R(f).J, R(f).J_std, R(f).omega_min, R(f).overshoot_loadoff, R(f).ss_error, ...
            strtok(R(f).worst_case), R(f).worst_stress, R(f).stalls);
end
fprintf('stall = speed below 50 rad/s. A controller qualifies if it never stalls; among those, the lowest mean J wins.\n');
if nargout == 0, clear R, end
end

% ---------------------------------------------------------------------------------------------------------------
function w = minSpeed(base, ns, sh, id, d, tau)
w = inf;   for s = 1:3, rr = runOne(base, s, ns, sh(s), id, d, tau);   w = min(w, rr.omega_min); end
end

function r = runOne(b, seed, nscale, nshift, ident, d, tau)
% one closed-loop run: random profile 'seed' (same generator as test_controller "comp"), noise scaled and shifted,
% plant parameters 'ident', optional throttle lag (d samples + first order tau) added to the controller output
c = b.c;   m = b.m;   o = b.opt;   Ts = 1e-3;
[Ad, Bd, Cd, Dd] = addLag(c, d, tau, Ts);
data = m.data;   t = data.meas.time(:);   N = numel(t);   T_end = floor(t(end));
wref = c.ISCS_Ty*ones(N,1);   ul = zeros(N,1);
rng(seed);
tk = 5;             while tk < t(end), wref(t >= tk) = max(o.w_low, 70 + 130*rand);  tk = tk + 8 + 7*rand; end
tk = 16 + 6*rand;   while tk < t(end), dd = 4 + 8*rand;  ul(t >= tk & t < tk+dd) = 1;  tk = tk + dd + 4 + 8*rand; end
noise = nscale*circshift(b.noise0, nshift);
c1 = ones(N,1);
data.meas.T_a.signals.values = b.Lm.theta_a_nom*c1;   data.meas.p_a.signals.values = b.Lm.p_a_nom*c1;
data.meas.T_m.signals.values = b.Lm.theta_m_nom*c1;   data.meas.p_e.signals.values = b.Lm.p_e_nom*c1;
data.meas.u_l.signals.values = ul;                    data.meas.P_l.signals.values = o.P_on*ul;
simcfg = m.simcfg;   simcfg.omega_meas = 0;
V = struct('data',data, 'init',b.init, 'ident',ident, 'constants',m.constants, 'limits',m.limits, 'simcfg',simcfg, ...
           'ISCS_Ad',Ad, 'ISCS_Bd',Bd, 'ISCS_Cd',Cd, 'ISCS_Dd',Dd, 'ISCS_Tu',c.ISCS_Tu, 'ISCS_Ty',c.ISCS_Ty, ...
           'Ts',Ts, 'T_end',T_end, 'ref_ts',timeseries(wref,t), 'noise_ts',timeseries(noise,t), 'du_min',o.du_min);
f = fieldnames(V);   for i = 1:numel(f), assignin('base', f{i}, V.(f{i})); end
out = sim(b.mdl);
r.t = out.omega_out.Time;   r.omega = out.omega_out.Data(:);
u = squeeze(out.u_out.Data);   if size(u,1) == 2, u = u.'; end
ua = interp1(out.u_out.Time, u(:,1), r.t);
wr = interp1(t, wref, r.t, 'previous');   lo = interp1(t, ul, r.t, 'previous');
ke = r.t >= 15;   kmax = 1/(1 - m.constants.k_zeta*c.ISCS_Tu(2,2)^2);
r.J_track = trapz(r.t(ke), 0.2*kmax^2.5*abs(r.omega(ke) - wr(ke)));
r.J_act   = sum(abs(diff(ua(ke))));   r.J = r.J_track + r.J_act;   r.omega_min = min(r.omega(ke));
% overshoot after load switch-off (reference constant from 1 s before to 3 s after)
r.ov = 0;   off = find(diff(lo) < -0.5);
for i = off(:)'
    k = r.t >= r.t(i) - 1 & r.t <= r.t(i) + 3;
    if r.t(i) > 15 && max(wr(k)) == min(wr(k)), r.ov = max(r.ov, max(r.omega(k & r.t >= r.t(i)) - wr(i+1))); end
end
% steady-state error: mean error over the last second of every segment (reference and load constant) longer than 6 s
r.sse = 0;   ch = [1; find(diff(wr) ~= 0 | diff(lo) ~= 0) + 1; numel(r.t)+1];
for i = 1:numel(ch)-1
    a = ch(i);  e = ch(i+1) - 1;
    if r.t(a) > 15 && r.t(e) - r.t(a) > 6, k = r.t >= r.t(e) - 1 & r.t <= r.t(e);  r.sse = max(r.sse, abs(mean(r.omega(k) - wr(k)))); end
end
r = rmfield(r, {'t','omega'});
end

function [Ad, Bd, Cd, Dd] = addLag(c, d, tau, Ts)
% throttle actuator lag: the normalized throttle output goes through d unit delays and a first-order lag tau
Ad = c.ISCS_Ad;  Bd = c.ISCS_Bd;  Cd = c.ISCS_Cd;  Dd = c.ISCS_Dd;
if d == 0 && tau == 0, return, end
n = size(Ad,1);   a = exp(-Ts/max(tau, eps));   k = d + 1;           % d shift states + 1 lag state
A = zeros(n+k);   A(1:n,1:n) = Ad;
if d > 0
    A(n+1,1:n) = Cd(1,:);                                        % z1(k+1) = y1(k)
    for i = 2:d, A(n+i, n+i-1) = 1; end                          % z_i(k+1) = z_{i-1}(k)
    A(n+k, n+d) = 1 - a;
else
    A(n+k,1:n) = (1 - a)*Cd(1,:);
end
A(n+k, n+k) = a;
B = [Bd; zeros(k,1)];   if d > 0, B(n+1) = Dd(1); else, B(n+k) = (1-a)*Dd(1); end
C = [zeros(1,n+k-1) 1; Cd(2,:) zeros(1,k)];   D = [0; Dd(2)];
Ad = A;  Bd = B;  Cd = C;  Dd = D;
end

function init = initFor(c, m, Mb, m3)
% plant initial state at the controller's nominal point (own linearizations first, then LinModels_bench.mat)
cand = Mb;
if isfile(fullfile(m3,'Jeremie','LinModels_Jeremie.mat'))
    J = load(fullfile(m3,'Jeremie','LinModels_Jeremie.mat')).LM;
    for i = 1:numel(J), cand(end+1).omega_e_nom = J(i).omega_e_nom; cand(end).delta_u_nom = J(i).delta_u_nom; cand(end).p_m_nom = J(i).p_m_nom; cand(end).Q_nom = J(i).Q_nom; end %#ok<AGROW>
end
d = abs([cand.omega_e_nom] - c.ISCS_Ty) + 1e3*([cand.delta_u_nom] ~= c.ISCS_Tu(2,2));   [~, k] = min(d);
init = m.init;   init.omega_e0 = c.ISCS_Ty;   init.p_m0 = cand(k).p_m_nom;   init.du_init = c.ISCS_Tu(2,2);   init.q_init = cand(k).Q_nom;
end

function file = findFile(f, m3)
if isfile(f), file = f; return, end
if isfile(which(f)), file = which(f); return, end
hit = dir(fullfile(m3, '*', f));
if numel(hit) ~= 1, error('controller file "%s" not found (or found several times) in the personal folders of M3 Controller', f); end
file = fullfile(hit.folder, hit.name);
end

function s = ifelse(c, a, b), if c, s = a; else, s = b; end, end

function m = getMain()
main
m = struct('constants',constants, 'limits',limits, 'simcfg',simcfg, 'init',init, 'data',data);
end
