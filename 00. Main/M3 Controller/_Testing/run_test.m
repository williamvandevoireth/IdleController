%% run_test.m  -  test one or several controllers on the nonlinear engine model: edit the settings, press Run
% Uses functions/test_controller.m and models/Bench_ISCS.slx. Nothing is written to disk.
% Controller files: competition format (ISCS_Ad, ISCS_Bd, ISCS_Cd, ISCS_Dd, ISCS_Tu, ISCS_Ty).
% A bare file name is found in any personal folder of M3 Controller; otherwise give the full path.

addpath(fileparts(mfilename('fullpath')));   % so that test_controller is found from any current folder

%% ---------------- 1. which controller(s) ----------------
files = {'controller1_William.mat'};                    % one file, or several for a comparison table, e.g.
                                                       % files = {'controller1_William.mat', 'controller2_William.mat', 'controller1_Leon.mat'};

%% ---------------- 2. reference profile and load ----------------
scenario = 'comp';       % 'comp'   random reference steps and random load switching (competition-like)
                         % 'step'   constant reference w_ref with five fixed load pulses
                         % 'custom' your own profile, defined below

% --- 'comp' ---
seed  = 1;               % random profile number: same seed = same profile for every controller (use 1...5)
w_low = 90;              % rad/s  lowest reference (a 1.2 kW load step at about 80 rad/s or lower stalls the model)

% --- 'step' ---
w_ref = 100;             % rad/s  constant reference (the load pulses are 20-35, 45-50, 60-80, 90-95, 100-115 s)

% --- 'custom' ---
t_ref   = [5   40   80];             % s       times at which the reference changes
w_vals  = [120 180  90];             % rad/s   reference from each time on (before the first time: the controller's nominal speed)
load_on = [20 35;  60 80];           % s       one row [start end] per load pulse

%% ---------------- 3. other settings ----------------
P_on   = 1200;           % W      load power when on
noise  = true;           % recorded measurement noise on the speed signal (false = noise-free, shows how much of J is noise)
du_min = -30;            % degCA  lower limit of the ignition retardation (assumption, the manual gives none)
t0     = 15;             % s      cost J is evaluated from here to the end (119 s)
showPlot = true;         % plot speed (with the load on the right axis), throttle and ignition (only for a single file)
separateFigures = true;  % one figure per plot (false: all four in one figure)

%% ---------------- run ----------------
args = {'scenario',scenario, 'seed',seed, 'w_low',w_low, 'w_ref',w_ref, 'P_on',P_on, 'noise',noise, 'du_min',du_min, 't0',t0, 'plot',showPlot, 'separate',separateFigures};
if strcmp(scenario, 'custom'), args = [args, {'t_ref',t_ref, 'w_vals',w_vals, 'load_on',load_on}]; end
if numel(files) == 1, files = files{1}; end
result = test_controller(files, args{:});              % result: struct with J, J_track, J_act, omega_min, signals ...
