%% LQGI_run.m  -  design, simulate and export the idle speed controller

addpath(strjoin(fullfile(fileparts(fileparts(mfilename('fullpath'))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'}), pathsep));   % milestone folders
main                                         % NOTE: main starts with 'clear', so it must stay at the top
mainDir = fileparts(which('LQGI_run'));      % this folder (M3 Controller)

%% ---------------- Tuning parameters (change only these) ----------------
r1   = 10;      % penalty on u_alpha      (small = use the throttle a lot)
r2   = 10;      % penalty on delta_u      (small = use the ignition a lot)
qw   = 100;     % state weight Q = qw*C'*C (large = faster speed control)
qobs = 1e-6;    % observer: small = fast (trust measurement), large = slow (trust model)
K1   = 1;       % scaling of the integrator state in the extension
Ts   = 1e-3;    % s sampling time (fixed by the competition)
saveFile = 'controller1.mat';   % use controller2.mat for the second controller

%% ---------------- Part 1: controller design ----------------
Lm = load(fullfile(mainDir,'LinModel_bench.mat'));
As = Lm.A;  Bs = Lm.B;  Cs = Lm.C;  Ds = Lm.D;           % normalized linear plant, 10 states
[Tb,Ab] = balance(As);  Bb = Tb\Bs;  Cb = Cs*Tb;         % rescale states (cond(As) is about 1e19)
fprintf('cond(A) %.3g -> %.3g after balancing\n', cond(As), cond(Ab));

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

% specs on the linear model: crossover <= 4 rad/s, min|1+L| >= 0.5
P  = ss(Ab, Bb, Cb, Ds);   w = logspace(-3,3,6000);
Lw = squeeze(freqresp(P*sysc, w));
fprintf('sizes Ad %s Bd %s Cd %s Dd %s\n', mat2str(size(ISCS_Ad)), mat2str(size(ISCS_Bd)), mat2str(size(ISCS_Cd)), mat2str(size(ISCS_Dd)));
fprintf('linear model: min|1+L| = %.3f (need >= 0.5), crossover = %.2f rad/s (hint <= 4), stable: %d\n', ...
        min(abs(1+Lw)), w(find(abs(Lw)<1,1)), isstable(feedback(P*sysc,1)));

% keep Controller_William.slx in sync (its blocks read these names from the model workspace)
if isfile(fullfile(mainDir,'Controller_William.slx'))
    load_system('Controller_William');  mws = get_param('Controller_William','ModelWorkspace');
    nm = {'A_lqg','B_lqg','C_lqg','D_lqg','Ae','Be','Ce','De','K1','ISCS_Ad','ISCS_Bd','ISCS_Cd','ISCS_Dd','ISCS_Tu','ISCS_Ty','Ts','K','r1','r2','qw','qobs'};
    for i = 1:numel(nm), assignin(mws, nm{i}, eval(nm{i})); end
    L = Lg;  assignin(mws,'L',L);
    fprintf('model workspace of Controller_William updated (model not saved: save it yourself if you want to keep it)\n');
end
