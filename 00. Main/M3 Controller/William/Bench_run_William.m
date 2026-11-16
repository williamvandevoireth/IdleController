%% Bench_run_William.m  -  closed-loop test of a competition file on the nonlinear plant
% Model: Bench_William.slx = discrete ISCS controller (matrices of the .mat file) + one sample delay + limits + Engine_Plant_CL.slx.
% The plant replays a load profile and constant ambient conditions that are built here; the speed measurement gets the
% high-frequency part of the speed recorded on the test bench (studentId_0002) as noise.
% Run LQGI_run_William.m first (it saves the controller file).

pd = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});   pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models')];   addpath(strjoin(pd(isfolder(pd)), pathsep));   % milestone folders with their functions and models
addpath(fileparts(mfilename('fullpath')));
main                                    % constants, limits, simcfg, data (main starts with 'clear')

%% ---------------- test settings ----------------
ctrlFile = 'controller1_William.mat';   % competition file to test
scenario = 'comp';      % 'comp'  random reference steps and random load switching (competition-like)
                        % 'step'  constant reference w_ref with load steps
seed     = 1;           % random seed of the 'comp' scenario
w_low    = 90;          % rad/s  lowest reference of the 'comp' scenario (a 1.2 kW load step at 70 rad/s cannot be survived, see README)
w_ref    = 100;         % rad/s  reference of the 'step' scenario
P_on     = 1200;        % W      load power when switched on
du_min   = -30;         % degCA  lower limit of the ignition retardation (assumption, the manual gives no value)
useNoise = true;        % recorded measurement noise on the speed signal
t0       = 15;          % s      evaluation starts here (after the start-up transient)

%% ---------------- controller and its nominal point ----------------
load(ctrlFile);                                          % ISCS_Ad, ISCS_Bd, ISCS_Cd, ISCS_Dd, ISCS_Tu, ISCS_Ty
S  = load('LinModels_bench.mat');   M = S.M;
k  = find([M.delta_u_nom] == ISCS_Tu(2,2) & abs([M.omega_e_nom] - ISCS_Ty) < 1, 1);   % operating point the controller is normalized with
Lm = load('LinModel_bench.mat');                         % ambient conditions used in the linearization
ident = S.ident;   Ts = 1e-3;
init.omega_e0 = M(k).omega_e_nom;  init.p_m0 = M(k).p_m_nom;  init.du_init = M(k).delta_u_nom;  init.q_init = M(k).Q_nom;   % start in equilibrium
simcfg.omega_meas = 0;

%% ---------------- scenario: reference, load, noise ----------------
t = data.meas.time(:);   N = numel(t);   T_end = floor(t(end));
wref = ISCS_Ty*ones(N,1);   ul = zeros(N,1);
switch scenario
    case 'comp'
        rng(seed);
        tk = 5;   while tk < t(end), wref(t >= tk) = max(w_low, 70 + 130*rand);  tk = tk + 8 + 7*rand; end
        tk = 16 + 6*rand;   while tk < t(end), d = 4 + 8*rand;  ul(t >= tk & t < tk+d) = 1;  tk = tk + d + 4 + 8*rand; end
    case 'step'
        wref(t >= 5) = w_ref;   on = [20 35; 45 50; 60 80; 90 95; 100 115];
        for j = 1:size(on,1), ul(t >= on(j,1) & t < on(j,2)) = 1; end
end
q  = load(fullfile(fileparts(which('main')),'..','..','ISCSData','studentId_0002.mat')).meas;
wq = q.omega_e.signals.values(:);   noise = wq(1:N) - movmean(wq(1:N), 200);     % recorded speed minus its 0.2 s moving average
if ~useNoise, noise = 0*noise; end
ref_ts = timeseries(wref, t);   noise_ts = timeseries(noise, t);

% the plant reads data.meas.* ("struct with time"): constant ambient conditions, our load profile
c1 = ones(N,1);
data.meas.T_a.signals.values = Lm.theta_a_nom*c1;   data.meas.p_a.signals.values = Lm.p_a_nom*c1;
data.meas.T_m.signals.values = Lm.theta_m_nom*c1;   data.meas.p_e.signals.values = Lm.p_e_nom*c1;
data.meas.u_l.signals.values = ul;                  data.meas.P_l.signals.values = P_on*ul;

%% ---------------- simulation ----------------
out = sim('Bench_William');

%% ---------------- evaluation (cost function of the manual, p. 39) ----------------
tt = out.omega_out.Time;   omega = out.omega_out.Data(:);
u  = squeeze(out.u_out.Data);   if size(u,1) == 2, u = u.'; end
u_alpha = interp1(out.u_out.Time, u(:,1), tt);   du = interp1(out.u_out.Time, u(:,2), tt);
omega_ref = interp1(t, wref, tt, 'previous');    load_on = interp1(t, ul, tt, 'previous');
ke = tt >= t0;                                    % evaluation window
kmax    = 1/(1 - constants.k_zeta*ISCS_Tu(2,2)^2);
J_track = trapz(tt(ke), 0.2*kmax^2.5*abs(omega(ke) - omega_ref(ke)));
J_act   = sum(abs(diff(u_alpha(ke))));            % integral of |d u_alpha/dt| dt for the sampled signal
J       = J_track + J_act;
fprintf('\n%s, scenario %s, %g s evaluated\n', ctrlFile, scenario, tt(end) - t0);
fprintf('J = %.0f  (tracking %.0f + throttle activity %.0f),  k_max^2.5 = %.3f\n', J, J_track, J_act, kmax^2.5);
fprintf('speed: min %.1f rad/s (stall limit 50), max %.1f rad/s, RMS error %.2f rad/s\n', min(omega(ke)), max(omega(ke)), rms(omega(ke) - omega_ref(ke)));
fprintf('u_alpha: %.2f ... %.2f %%,  delta_u: %.1f ... %.1f degCA (limits %g ... 0)\n', min(u_alpha(ke)), max(u_alpha(ke)), min(du(ke)), max(du(ke)), du_min);
if any(omega < 50), warning('Speed dropped below 50 rad/s: the engine would stall on the test bench.'); end

figure(20); clf
subplot(4,1,1); plot(tt, omega_ref, 'k--', tt, omega, 'r'); grid on; ylabel('\omega_e [rad/s]'); legend('ref','sim','Location','best'); title(sprintf('%s, scenario %s', strrep(ctrlFile,'_','\_'), scenario))
subplot(4,1,2); plot(tt, u_alpha); grid on; ylabel('u_\alpha [%]')
subplot(4,1,3); plot(tt, du); grid on; ylabel('\Delta u_\zeta [degCA]')
subplot(4,1,4); plot(tt, P_on*load_on); grid on; ylabel('P_l [W]'); xlabel('t [s]')
