%% LQGI_run_William.m  -  LQGI idle speed controller: design, robustness check, export of the competition file
% The controller is designed on the linear model of ONE operating point of LinModels_bench.mat, with the load switched on:
% the load is a constant-power load, T_l = P_l/(omega_e*eta_gen), so its torque rises when the speed falls. Linearized, this adds
% + P_l/(omega_e^2*eta_gen*Theta_e) to the speed equation and makes the plant slower or even unstable at low speed
% (pole at +0.55 1/s at 70 rad/s with 1.2 kW). A design at 140 rad/s without load does not stabilize that case.

pd = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});   pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models'), fullfile(pd,'_Models'), fullfile(pd,'_Testing')];   addpath(strjoin(pd(isfolder(pd)), pathsep));   % milestone folders with their functions and models
addpath(fileparts(mfilename('fullpath')));
main
mainDir = fileparts(which('LQGI_run_William'));

%% ---------------- tuning parameters ----------------
ctrl = 1;       % 1 = lowest cost J in the tests, 2 = largest stability margin, uses the ignition less
switch ctrl
    case 1,  w_design = 100;  du_nom = -15;  P_design = 1800;  r1 = 3;   r2 = 1;   qw = 100;  qobs = 1e-6;
    case 2,  w_design = 70;   du_nom = -15;  P_design = 1800;  r1 = 10;  r2 = 10;  qw = 100;  qobs = 1e-6;
end
% w_design  rad/s  design speed (70, 100, 140 or 200: the points of LinModels_bench.mat)
% du_nom    degCA  nominal ignition retardation (-7 or -15); -15 gives the ignition authority, at the cost k_max^2.5 = 1.14 in J
% P_design  W      load assumed on in the design model (larger = more robust at low speed)
% r1, r2           penalty on u_alpha, delta_u (small = use a lot of it)
% qw               state weight Q = qw*C'*C (large = faster speed control)
% qobs             observer: small = fast (trust measurement), large = slow (trust model, less throttle noise)
K1   = 1;       % scaling of the integrator state in the extension
Ts   = 1e-3;    % s sampling time (fixed by the competition)
saveFile = sprintf('controller%d_William.mat', ctrl);

%% ---------------- design model ----------------
S  = load('LinModels_bench.mat');   M = S.M;             % linear models at 70/100/140/200 rad/s, delta_u_nom -7/-15
k  = find([M.delta_u_nom] == du_nom & abs([M.omega_e_nom] - w_design) < 1, 1);
if isempty(k), error('No linear model at %g rad/s, delta_u_nom %g in LinModels_bench.mat.', w_design, du_nom); end
Lm = M(k);
As = Lm.A;  Bs = Lm.B;  Cs = Lm.C;  Ds = Lm.D;           % normalized linear plant, 10 states
iw = find(abs(Cs) > 0.5, 1);                             % index of the speed state (y = normalized omega_e)
kl = @(P,w) P/(w^2*S.eta_gen*S.Theta_e);                 % 1/s, linearized constant-power load
As(iw,iw) = As(iw,iw) + kl(P_design, Lm.omega_e_nom);
[Tb,Ab] = balance(As);  Bb = Tb\Bs;  Cb = Cs*Tb;         % rescale states (cond(As) is about 1e19)
fprintf('design model: %.0f rad/s, delta_u_nom %g degCA, load %g W; cond(A) %.3g -> %.3g after balancing\n', Lm.omega_e_nom, du_nom, P_design, cond(As), cond(Ab));

%% ---------------- controller design ----------------
% plant extension: integrator on u_alpha
Ae = 0;  Be = [1 0];  Ce = [K1; 0];  De = eye(2);
n  = size(Ab,1);
A  = [Ab, Bb*Ce; zeros(1,n), Ae];   B = [Bb*De; Be];   C = [Cb, Ds*Ce];

% feedback gain and observer gain
K  = lqr(A, B, qw*(C'*C), diag([r1 r2]));
Lg = lqr(A', C', B*B', qobs)';

% controller matrices, Dc = 0
nx = size(A,1);  Dc = zeros(2,1);
A_lqg = A - B*K - Lg*C;
B_lqg = -Lg;
C_lqg = -K;
D_lqg = zeros(2,1);

ISCS_Ac = [Ae, -Be*K; zeros(nx,1), A_lqg];
ISCS_Bc = [Be*Dc; -Lg];
ISCS_Cc = [Ce, -De*K];
ISCS_Dc = De*Dc;
sysc = ss(ISCS_Ac, ISCS_Bc, ISCS_Cc, ISCS_Dc);

% discretization (Tustin) and normalization matrices
sysd = c2d(sysc, Ts, 'tustin');
ISCS_Ad = sysd.A;  ISCS_Bd = sysd.B;  ISCS_Cd = sysd.C;  ISCS_Dd = sysd.D;
ISCS_Tu = diag([Lm.u_alpha_nom, Lm.delta_u_nom]);
ISCS_Ty = Lm.omega_e_nom;
fprintf('sizes Ad %s Bd %s Cd %s Dd %s\n', mat2str(size(ISCS_Ad)), mat2str(size(ISCS_Bd)), mat2str(size(ISCS_Cd)), mat2str(size(ISCS_Dd)));

%% ---------------- robustness over the operating range ----------------
% Loop gain of this controller with the plant at every speed of LinModels_bench.mat, load off and on (1.2 kW).
% Required: stable everywhere, min|1+L| >= 0.5 (manual p. 33). Hint of the manual: crossover <= 4 rad/s (noise).
w = logspace(-2, 3, 3000);
fprintf('\n  omega   load | plant max Re | stable  min|1+L|  crossover\n');
allStable = true;  muMin = inf;
for j = find([M.delta_u_nom] == du_nom)
    for P = [0 1200]
        Aj = M(j).A;  Aj(iw,iw) = Aj(iw,iw) + kl(P, M(j).omega_e_nom);
        Sc = diag([ISCS_Tu(1,1)/M(j).u_alpha_nom, ISCS_Tu(2,2)/M(j).delta_u_nom]) * (M(j).omega_e_nom/ISCS_Ty);   % controller units -> units of model j
        Lj = ss(Aj, M(j).B, M(j).C, M(j).D) * Sc * sysc;
        Lw = squeeze(freqresp(Lj, w));  mu = min(abs(1+Lw));  wc = w(find(abs(Lw) >= 1, 1, 'last'));  st = isstable(feedback(Lj,1));
        fprintf('  %5.0f  %5.0f | %+11.2f  | %4d    %7.3f   %6.2f rad/s\n', M(j).omega_e_nom, P, max(real(eig(Aj))), st, mu, wc);
        allStable = allStable && st;  muMin = min(muMin, mu);
    end
end
fprintf('all cases stable: %d, worst min|1+L| = %.3f (need >= 0.5)\n', allStable, muMin);

%% ---------------- competition file ----------------
save(fullfile(mainDir, saveFile), 'ISCS_Ad', 'ISCS_Bd', 'ISCS_Cd', 'ISCS_Dd', 'ISCS_Tu', 'ISCS_Ty');
fprintf('saved %s (test it with run_test or test_controller in M3 Controller/_Testing)\n', saveFile);
