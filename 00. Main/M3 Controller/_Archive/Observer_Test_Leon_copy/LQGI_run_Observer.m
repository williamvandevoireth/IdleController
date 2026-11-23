%% LQGI_run_<Name>.m  -  starting point for your own idle speed controller (team template)
% How to use:
%   1. This file lives in your personal folder "M3 Controller/<Name>/". Work only in that folder.
%   2. Change the tuning parameters below, or replace the design section by your own method.
%      Whatever you do, the result must be the six competition matrices (manual p. 39).
%   3. Run the script. It saves controller1_<Name>.mat in your folder and tests it on the nonlinear plant
%      with the shared bench "M4 Competition/test_controller.m" (cost J, minimum speed, plot).
%   More tests:  test_controller(file, scenario="step", w_ref=140),  test_controller(file, seed=2),
%                test_controller({fileA, fileB})   compares several controllers on the same scenario.
%
% Background (README, section 7): the load is a constant-power load, T_l = P_l/(omega_e*eta_gen), so its torque rises when the
% speed falls. With 1.2 kW on, the plant is open-loop unstable at 70 rad/s. Design on a low-speed model with the load on,
% and check all operating points, otherwise the controller stalls at low reference speeds.

pd = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});   pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models'), fullfile(pd,'_Models'), fullfile(pd,'_Testing')];   addpath(strjoin(pd(isfolder(pd)), pathsep));   % milestone folders with their functions and models
main                                                     % workspace of the project (main starts with 'clear')
myDir = fileparts(mfilename('fullpath'));
observerRoot = fileparts(myDir);
myName = 'Observer';

%% ---------------- tuning parameters (change these) ----------------
w_design = 100;     % rad/s  design speed: 70, 100, 140 or 200 (operating points of LinModels_bench.mat)
du_nom   = -15;     % degCA  nominal ignition retardation: -7 or -15. More retardation = more ignition authority, but k_max^2.5 in J rises (1.03 at -7, 1.14 at -15)
P_design = 1800;    % W      load assumed on in the design model (larger = more robust at low speed, 0 = no load)
r1   = 3;           % penalty on u_alpha   (small = use the throttle a lot)
r2   = 1;           % penalty on delta_u   (small = use the ignition a lot)
qw   = 100;         % state weight Q = qw*C'*C (large = faster speed control)
qobs = 1e-6;        % observer: small = fast (trust measurement), large = slow (trust model, less throttle noise)
K1   = 1;           % scaling of the integrator state in the extension
Ts   = 1e-3;        % s      sampling time (fixed by the competition)
saveFile = sprintf('controller1_%s.mat', myName);        % use controller2_... for a second controller

%% ---------------- design model ----------------
S  = load('LinModels_bench.mat');   M = S.M;             % linear models at 70/100/140/200 rad/s, delta_u_nom -7/-15
k  = find([M.delta_u_nom] == du_nom & abs([M.omega_e_nom] - w_design) < 1, 1);
if isempty(k), error('No linear model at %g rad/s, delta_u_nom %g in LinModels_bench.mat.', w_design, du_nom); end
Lm = M(k);
As = Lm.A;  Bs = Lm.B;  Cs = Lm.C;  Ds = Lm.D;           % normalized linear plant, 10 states
iw = find(abs(Cs) > 0.5, 1);                             % index of the speed state (y = normalized omega_e)
kl = @(P,w) P/(w^2*S.eta_gen*S.Theta_e);                 % 1/s, linearized constant-power load
As(iw,iw) = As(iw,iw) + kl(P_design, Lm.omega_e_nom);
[Tb,Ab] = balance(As);  Bb = Tb\Bs;  Cb = Cs*Tb;         % rescale the states

%% ---------------- controller design: LQGI (manual p. 33-36) ----------------
% plant extension: integrator on u_alpha
Ae = 0;  Be = [1 0];  Ce = [K1; 0];  De = eye(2);
n  = size(Ab,1);
A  = [Ab, Bb*Ce; zeros(1,n), Ae];   B = [Bb*De; Be];   C = [Cb, Ds*Ce];

% feedback gain and observer gain
K  = lqr(A, B, qw*(C'*C), diag([r1 r2]));
Lg = lqr(A', C', B*B', qobs)';


%% ---------------- separate observer matrices ----------------
% Observer: x_hat_dot = Aobs*x_hat + Bobs*u + Lg*y
% Same observer gain as used in the LQGI controller

Aobs = A - Lg*C;
Bobs = [B, Lg];

Cobs = eye(size(A,1));
Dobs = zeros(size(A,1), size(Bobs,2));

% Output mapping: observer state estimate -> normalized engine speed
Obs_Cy = C;
%% DEBUG: Inspect observer output matrix

fprintf('\n========== OBSERVER OUTPUT MATRIX ==========\n');

fprintf('Size of Obs_Cy: %s\n',mat2str(size(Obs_Cy)));

disp('Obs_Cy = ');
disp(Obs_Cy);

fprintf('Sum of Obs_Cy coefficients: %.10f\n',sum(Obs_Cy));

fprintf('ISCS_Ty = %.10f\n',Lm.omega_e_nom);

fprintf('============================================\n');
% Display dimensions
disp('Observer matrices:')
disp(['Aobs: ', mat2str(size(Aobs))])
disp(['Bobs: ', mat2str(size(Bobs))])
disp(['Cobs: ', mat2str(size(Cobs))])
disp(['Dobs: ', mat2str(size(Dobs))])


% controller matrices (Dc = 0), input e = (omega_ref - omega_e)/omega_e_nom
nx = size(A,1);  Dc = zeros(2,1);
ISCS_Ac = [Ae, -Be*K; zeros(nx,1), A - B*K - Lg*C];
ISCS_Bc = [Be*Dc; -Lg];
ISCS_Cc = [Ce, -De*K];
ISCS_Dc = De*Dc;
sysc = ss(ISCS_Ac, ISCS_Bc, ISCS_Cc, ISCS_Dc);

% discretization (Tustin) and normalization matrices
sysd = c2d(sysc, Ts, 'tustin');
ISCS_Ad = sysd.A;  ISCS_Bd = sysd.B;  ISCS_Cd = sysd.C;  ISCS_Dd = sysd.D;
ISCS_Tu = diag([Lm.u_alpha_nom, Lm.delta_u_nom]);
ISCS_Ty = Lm.omega_e_nom;

%% ---------------- robustness over the operating range ----------------
% Loop gain with the plant at every speed of LinModels_bench.mat, load off and on (1.2 kW).
% Required: stable everywhere and min|1+L| >= 0.5 (manual p. 33). Hint of the manual: crossover <= 4 rad/s (noise).
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

%% ---------------- competition file and closed-loop test ----------------
save(fullfile(myDir, saveFile), ...
    'ISCS_Ad', 'ISCS_Bd', 'ISCS_Cd', 'ISCS_Dd', 'ISCS_Tu', 'ISCS_Ty', 'Obs_Cy');
fprintf('saved %s\n', fullfile(myName, saveFile));
test_controller(fullfile(myDir, saveFile));              % shared bench in M4 Competition
