%% run_live.m  -  interactive test: edit the settings, press Run, a window opens with live graphs, sliders and buttons
% Uses functions/live_test.m and models/Bench_Live.slx. Nothing is written to disk. Close the window or press "Stop and close" to end.
%
% In the window:  - slider "Reference speed"        the speed the controller should hold (70-200 rad/s); the controller sets the throttle itself
%                 - button "Load"                   electrical load on / off (the 1.2 kW water boiler)
%                 - tick box "Random load"          load switches on and off at random times
%                 - tick boxes "Show in graphs"     speed, reference, speed error, throttle, ignition, manifold pressure, engine torque,
%                                                   load torque, air mass flows, load power
%% Select the personal live_test.m in M3 Controller/Leon

%% Select the Observer_Test live simulation

projectRoot = 'C:\Users\leont\Documents\ETH\Engine Systems\ISCS\IdleController\00. Main';

observerRoot = fullfile(projectRoot,'Observer_Test');

functionsFolder = fullfile(observerRoot,'functions');
modelsFolder = fullfile(observerRoot,'models');

% Temporarily use the project root as current folder.
oldFolder = pwd;
folderCleanup = onCleanup(@() cd(oldFolder));
cd(projectRoot);

% Add the new Observer_Test folders.
addpath(functionsFolder,'-begin');
addpath(modelsFolder,'-begin');

% Clear MATLAB's cached function and refresh the file system cache.
clear live_test
clear LGQI_run_Observer
rehash

% Verify the selected file before starting.
expectedFile = fullfile(functionsFolder,'live_test.m');
selectedFile = which('live_test');

fprintf('Selected live_test: %s\n',selectedFile);

if ~strcmpi(selectedFile,expectedFile)
    error('Wrong live_test.m selected! Expected: %s',expectedFile);
end

% Temporarily use the project root as current folder.
% This prevents a live_test.m in the current folder from taking priority.
oldFolder = pwd;
folderCleanup = onCleanup(@() cd(oldFolder));
cd(projectRoot);

% Put the personal functions folder first.
addpath(m3Functions, '-begin');

% Clear MATLAB's cached function and refresh the file system cache.
clear live_test
rehash

% Verify the selected file before starting.
expectedFile = fullfile(m3Functions, 'live_test.m');
selectedFile = which('live_test');

fprintf('Selected live_test: %s\n', selectedFile);

if ~strcmpi(selectedFile, expectedFile)
    error('Wrong live_test.m selected! Expected: %s', expectedFile);
end

%% ---------------- settings ----------------
ctrlFile = fullfile(observerRoot,'controller1_Leon.mat');
P_on     = 1200;                         % W      load power when the load is on
rand_on  = [4 12];                       % s      random load: how long it stays on  (min max)
rand_off = [4 12];                       % s      random load: how long it stays off (min max)
du_min   = -30;                          % degCA  lower limit of the ignition retardation (assumption, the manual gives none)
window   = 40;                           % s      width of the sliding time window of the graphs

%% ---------------- run ----------------
live_test(ctrlFile, P_on=P_on, rand_on=rand_on, rand_off=rand_off, du_min=du_min, window=window);
