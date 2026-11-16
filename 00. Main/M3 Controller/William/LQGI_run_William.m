%% LQGI_run_William.m  -  design, simulate and export the idle speed controller

addpath(strjoin(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'}), pathsep));   % milestone folders
addpath(fileparts(mfilename('fullpath')));
main
mainDir = fileparts(which('LQGI_run_William'));

%% ---------------- tuning parameters ----------------
r1   = 10;      % penalty on u_alpha      (small = use the throttle a lot)
r2   = 10;      % penalty on delta_u      (small = use the ignition a lot)
qw   = 100;     % state weight Q = qw*C'*C (large = faster speed control)
qobs = 1e-6;    % observer: small = fast (trust measurement), large = slow (trust model)
K1   = 1;       % scaling of the integrator state in the extension
Ts   = 1e-3;    % s sampling time (fixed by the competition)
saveFile = 'controller1_William.mat';

%% ---------------- controller design ----------------
Lm = load('LinModel_bench.mat');                         % shared linear model
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
