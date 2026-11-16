function r = test_controller(ctrlFile, opt)
%TEST_CONTROLLER  Closed-loop test of a competition controller file on the nonlinear engine model.
%
%   test_controller('controller1.mat')                       competition-like run (random reference and load)
%   test_controller(file, scenario="step", w_ref=140)        constant reference with five load steps
%   test_controller(file, seed=2, noise=false, plot=false)   other options, see below
%   r = test_controller(file)                                returns the numbers and the signals
%   test_controller({'a.mat','b.mat'})                       several files on the same scenario: one line per file, no plots
%
%   The file must contain the competition matrices (manual p. 39):
%   ISCS_Ad (12x12), ISCS_Bd (12x1), ISCS_Cd (2x12), ISCS_Dd (2x1), ISCS_Tu = diag(u_alpha_nom, delta_u_nom), ISCS_Ty = omega_e_nom.
%   Convention (inferred from manual Fig. 2.15): controller input = (omega_ref - omega_e)/ISCS_Ty,
%   plant input = ISCS_Tu*(1 + controller output), i.e. normalized deviations from the nominal point.
%
%   Model: Bench_ISCS.slx = discrete controller -> one sample delay -> limits -> Engine_Plant_CL.slx (bench-identified parameters).
%   Ambient conditions are constant, the load is switched by the scenario, the speed measurement gets recorded bench noise.
%   No file is written. The variables the models read are put into the base workspace (data, init, ident, constants, limits,
%   simcfg, ISCS_*, Ts, T_end, ref_ts, noise_ts, du_min); nothing else there is touched.
%
%   Options (name=value):
%     scenario  "comp" (default) random reference steps and random load switching | "step" constant reference with load steps
%               | "custom" your own profile: t_ref [s], w_vals [rad/s], load_on (rows [start end] in s), e.g.
%                 test_controller(f, scenario="custom", t_ref=[5 40 80], w_vals=[120 180 90], load_on=[20 35; 60 80])
%     seed      1        random seed of "comp"
%     w_low     90       rad/s, lowest reference of "comp" (a 1.2 kW load step near 70 rad/s cannot be survived by any controller, see README)
%     w_ref     100      rad/s, reference of "step"
%     P_on      1200     W, load power when on
%     du_min    -30      degCA, lower limit of the ignition retardation (assumption, the manual gives none)
%     noise     true     add the noise recorded on the test bench to the speed measurement
%     t0        15       s, the cost is evaluated from here (after the start-up transient) to the end (119 s)
%     plot      true     plots of speed (with the load power on the right axis), throttle and ignition
%     separate  true     one figure per plot (false: all four in one figure)
arguments
    ctrlFile
    opt.scenario (1,1) string {mustBeMember(opt.scenario, ["comp","step","custom"])} = "comp"
    opt.t_ref    (1,:) double = []           % "custom": times [s] at which the reference changes
    opt.w_vals   (1,:) double = []           % "custom": reference [rad/s] from each time in t_ref on (before the first time: the controller's nominal speed)
    opt.load_on  (:,2) double = zeros(0,2)   % "custom": load pulses, one row [start end] in s per pulse
    opt.seed     (1,1) double = 1
    opt.w_low    (1,1) double = 90
    opt.w_ref    (1,1) double = 100
    opt.P_on     (1,1) double = 1200
    opt.du_min   (1,1) double = -30
    opt.noise    (1,1) logical = true
    opt.t0       (1,1) double = 15
    opt.plot     (1,1) logical = true
    opt.separate (1,1) logical = true
end
here = fileparts(mfilename('fullpath'));
pd = fullfile(fileparts(fileparts(here)), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});   pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models')];   addpath(strjoin(pd(isfolder(pd)), pathsep));   % milestone folders with their functions and models

if iscell(ctrlFile) || (isstring(ctrlFile) && numel(ctrlFile) > 1)       % several files: same scenario, table
    files = cellstr(ctrlFile);  opt.plot = false;
    for i = 1:numel(files), r(i) = runOne(files{i}, opt, false); end %#ok<AGROW>
    fprintf('\nscenario %s%s, noise %d, evaluated %g-%g s\n', opt.scenario, scenTxt(opt), opt.noise, opt.t0, r(1).t(end));
    fprintf('%-34s %8s %9s %9s %8s %8s %6s\n', 'file', 'J', 'tracking', 'throttle', 'min w', 'RMS err', 'stall');
    for i = 1:numel(files)
        [~, nm, ex] = fileparts(files{i});
        fprintf('%-34s %8.0f %9.0f %9.0f %8.1f %8.2f %6d\n', [nm ex], r(i).J, r(i).J_track, r(i).J_act, r(i).omega_min, r(i).rms_error, r(i).stall);
    end
else
    r = runOne(char(ctrlFile), opt, true);
end
if nargout == 0, clear r, end
end

% ---------------------------------------------------------------------------------------------------------------------
function r = runOne(ctrlFile, opt, verbose)
%% controller file
if ~isfile(ctrlFile) && isfile(which(ctrlFile)), ctrlFile = which(ctrlFile); end
if ~isfile(ctrlFile)                              % a bare file name is searched in the personal folders of M3 Controller
    [pp, nm, ex] = fileparts(ctrlFile);
    if isempty(pp)
        hit = dir(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'M3 Controller', '*', [nm ex]));
        if numel(hit) == 1, ctrlFile = fullfile(hit.folder, hit.name);
        elseif numel(hit) > 1, error('"%s" exists in several personal folders (%s). Give the full path.', [nm ex], strjoin({hit.folder}, ', ')); end
    end
end
if ~isfile(ctrlFile), error('Controller file "%s" not found. Give the full path, or just the file name if it is in a personal folder of M3 Controller.', ctrlFile); end
c = load(ctrlFile);
need = {'ISCS_Ad',[12 12]; 'ISCS_Bd',[12 1]; 'ISCS_Cd',[2 12]; 'ISCS_Dd',[2 1]; 'ISCS_Tu',[2 2]; 'ISCS_Ty',[1 1]};
for i = 1:size(need,1)
    if ~isfield(c, need{i,1}), error('%s: variable %s is missing.', ctrlFile, need{i,1}); end
    if ~isequal(size(c.(need{i,1})), need{i,2}), error('%s: %s is %s, expected %s.', ctrlFile, need{i,1}, mat2str(size(c.(need{i,1}))), mat2str(need{i,2})); end
end
if c.ISCS_Tu(2,2) > 0, warning('ISCS_Tu(2,2) = delta_u_nom is positive; the ignition retardation must be negative.'); end

%% plant data: structs of main.m, bench identification, nominal operating point of the controller
m  = getMain();                                   % constants, limits, simcfg, init, data (as built by main.m)
S  = load('LinModels_bench.mat');   M = S.M;      % operating points 70/100/140/200 rad/s, delta_u_nom -7/-15
Lm = load('LinModel_bench.mat');                  % ambient conditions of the linearization
ident = S.ident;   Ts = 1e-3;
% equilibrium of the plant at the controller's nominal point (p_m and mdot_beta/omega_e), taken from the linearization sweep
sameDu = find([M.delta_u_nom] == c.ISCS_Tu(2,2));
if isempty(sameDu), sameDu = 1:numel(M); end
[dw, j] = min(abs([M(sameDu).omega_e_nom] - c.ISCS_Ty));   k = sameDu(j);
if dw > 1 || M(k).delta_u_nom ~= c.ISCS_Tu(2,2)
    warning('No stored operating point at %.1f rad/s, %g degCA. Starting from the nearest one (%.0f rad/s, %g degCA); expect a start-up transient before t0.', ...
            c.ISCS_Ty, c.ISCS_Tu(2,2), M(k).omega_e_nom, M(k).delta_u_nom);
end
init = m.init;   init.omega_e0 = c.ISCS_Ty;   init.p_m0 = M(k).p_m_nom;   init.du_init = c.ISCS_Tu(2,2);   init.q_init = M(k).Q_nom;
simcfg = m.simcfg;   simcfg.omega_meas = 0;       % the plant simulates its own speed

%% scenario: reference, load, noise
data = m.data;   t = data.meas.time(:);   N = numel(t);   T_end = floor(t(end));
wref = c.ISCS_Ty*ones(N,1);   ul = zeros(N,1);
switch opt.scenario
    case "comp"
        rng(opt.seed);
        tk = 5;             while tk < t(end), wref(t >= tk) = max(opt.w_low, 70 + 130*rand);  tk = tk + 8 + 7*rand; end
        tk = 16 + 6*rand;   while tk < t(end), d = 4 + 8*rand;  ul(t >= tk & t < tk+d) = 1;  tk = tk + d + 4 + 8*rand; end
    case "step"
        wref(t >= 5) = opt.w_ref;   on = [20 35; 45 50; 60 80; 90 95; 100 115];
        for i = 1:size(on,1), ul(t >= on(i,1) & t < on(i,2)) = 1; end
    case "custom"
        if numel(opt.t_ref) ~= numel(opt.w_vals), error('t_ref and w_vals must have the same length.'); end
        for i = 1:numel(opt.t_ref), wref(t >= opt.t_ref(i)) = opt.w_vals(i); end
        for i = 1:size(opt.load_on,1), ul(t >= opt.load_on(i,1) & t < opt.load_on(i,2)) = 1; end
end
q  = load(fullfile(fileparts(which('main')), '..', '..', 'ISCSData', 'studentId_0002.mat')).meas;
wq = q.omega_e.signals.values(:);   noise = wq(1:N) - movmean(wq(1:N), 200);      % recorded speed minus its 0.2 s moving average
if ~opt.noise, noise = 0*noise; end
c1 = ones(N,1);                                   % the plant reads data.meas.* ("struct with time")
data.meas.T_a.signals.values = Lm.theta_a_nom*c1;   data.meas.p_a.signals.values = Lm.p_a_nom*c1;
data.meas.T_m.signals.values = Lm.theta_m_nom*c1;   data.meas.p_e.signals.values = Lm.p_e_nom*c1;
data.meas.u_l.signals.values = ul;                  data.meas.P_l.signals.values = opt.P_on*ul;

%% simulation (the models read these names from the base workspace)
V = struct('data',data, 'init',init, 'ident',ident, 'constants',m.constants, 'limits',m.limits, 'simcfg',simcfg, ...
           'ISCS_Ad',c.ISCS_Ad, 'ISCS_Bd',c.ISCS_Bd, 'ISCS_Cd',c.ISCS_Cd, 'ISCS_Dd',c.ISCS_Dd, 'ISCS_Tu',c.ISCS_Tu, 'ISCS_Ty',c.ISCS_Ty, ...
           'Ts',Ts, 'T_end',T_end, 'ref_ts',timeseries(wref,t), 'noise_ts',timeseries(noise,t), 'du_min',opt.du_min);
f = fieldnames(V);   for i = 1:numel(f), assignin('base', f{i}, V.(f{i})); end
out = sim('Bench_ISCS');

%% evaluation: cost function of the manual (p. 39)
r.file = ctrlFile;   r.t = out.omega_out.Time;   r.omega = out.omega_out.Data(:);
u = squeeze(out.u_out.Data);   if size(u,1) == 2, u = u.'; end
r.u_alpha = interp1(out.u_out.Time, u(:,1), r.t);   r.delta_u = interp1(out.u_out.Time, u(:,2), r.t);
r.omega_ref = interp1(t, wref, r.t, 'previous');    r.load_on = interp1(t, ul, r.t, 'previous');
ke = r.t >= opt.t0;
r.kmax    = 1/(1 - m.constants.k_zeta*c.ISCS_Tu(2,2)^2);
r.J_track = trapz(r.t(ke), 0.2*r.kmax^2.5*abs(r.omega(ke) - r.omega_ref(ke)));
r.J_act   = sum(abs(diff(r.u_alpha(ke))));        % integral of |d u_alpha/dt| dt for the sampled signal
r.J       = r.J_track + r.J_act;
r.omega_min = min(r.omega(ke));   r.omega_max = max(r.omega(ke));   r.rms_error = rms(r.omega(ke) - r.omega_ref(ke));
r.stall   = any(r.omega < 50);

if verbose
    [~, nm, ex] = fileparts(ctrlFile);
    fprintf('\n%s%s: scenario %s%s, noise %d, evaluated %g-%g s\n', nm, ex, opt.scenario, scenTxt(opt), opt.noise, opt.t0, r.t(end));
    fprintf('  J = %.0f  (tracking %.0f + throttle activity %.0f),  k_max^2.5 = %.3f\n', r.J, r.J_track, r.J_act, r.kmax^2.5);
    fprintf('  speed: min %.1f rad/s (stall limit 50), max %.1f rad/s, RMS error %.2f rad/s\n', r.omega_min, r.omega_max, r.rms_error);
    fprintf('  u_alpha: %.2f ... %.2f %%,  delta_u: %.1f ... %.1f degCA (limits %g ... 0)\n', min(r.u_alpha(ke)), max(r.u_alpha(ke)), min(r.delta_u(ke)), max(r.delta_u(ke)), opt.du_min);
    if r.stall, warning('Speed dropped below 50 rad/s: the engine would stall on the test bench.'); end
end
if opt.plot
    [~, nm] = fileparts(ctrlFile);   tt = sprintf('%s, scenario %s, J = %.0f', strrep(nm,'_','\_'), opt.scenario, r.J);
    % one figure per plot (opt.separate = false: all in one figure); the load power is drawn in the speed figure (right axis)
    P = {'Speed',    @() plot(r.t, r.omega_ref, 'k--', r.t, r.omega, 'r'),   '\omega_e [rad/s]',          {'reference','speed'}
         'Throttle', @() plot(r.t, r.u_alpha),                               'u_\alpha [%]',              {}
         'Ignition', @() plot(r.t, r.delta_u),                               '\Delta u_\zeta [degCA]',   {} };
    n = size(P,1);
    for i = 1:n
        if opt.separate, figure('Name', ['test_controller: ' nm ' - ' P{i,1}]); clf; else, if i == 1, figure('Name', ['test_controller: ' nm]); clf; end, subplot(n,1,i); end
        if i == 1, yyaxis left; end
        P{i,2}();   grid on;   ylabel(P{i,3});
        if i == 1                                                                % load power on the right axis of the speed plot
            yyaxis right;   stairs(r.t, opt.P_on*r.load_on, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);   ylabel('P_l [W]');   ylim([0 3*opt.P_on]);
            ax = gca;   ax.YAxis(2).Color = [0.4 0.4 0.4];   yyaxis left;   ax.YAxis(1).Color = 'k';   legend({'reference','speed','load'}, 'Location', 'best');
        elseif ~isempty(P{i,4}), legend(P{i,4}, 'Location', 'best');
        end
        if opt.separate || i == 1, title(tt); end
        if opt.separate || i == n, xlabel('t [s]'); end
    end
end
end

function s = scenTxt(opt)
switch opt.scenario
    case "comp",   s = sprintf(' (seed %d, references >= %g rad/s)', opt.seed, opt.w_low);
    case "step",   s = sprintf(' (reference %g rad/s)', opt.w_ref);
    otherwise,     s = sprintf(' (reference steps at %s s, %d load pulses)', mat2str(opt.t_ref), size(opt.load_on,1));
end
end

function m = getMain()
% runs main.m inside this function, so its 'clear' does not touch the caller's or the base workspace
main
m = struct('constants',constants, 'limits',limits, 'simcfg',simcfg, 'init',init, 'data',data);
end
