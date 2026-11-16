%% run_live.m  -  interactive test: edit the settings, press Run, a window opens with live graphs, sliders and buttons
% Uses functions/live_test.m and models/Bench_Live.slx. Nothing is written to disk. Close the window or press "Stop and close" to end.
%
% In the window:  - slider "Reference speed"        the speed the controller should hold (70-200 rad/s); the controller sets the throttle itself
%                 - button "Load"                   electrical load on / off (the 1.2 kW water boiler)
%                 - tick box "Random load"          load switches on and off at random times
%                 - tick boxes "Show in graphs"     speed, reference, speed error, throttle, ignition, manifold pressure, engine torque,
%                                                   load torque, air mass flows, load power
addpath(fullfile(fileparts(mfilename('fullpath')),'functions'));             % so that live_test is found from any current folder

%% ---------------- settings ----------------
ctrlFile = 'controller1_William.mat';    % competition-format file; a bare name is found in any personal folder of M3 Controller
P_on     = 1200;                         % W      load power when the load is on
rand_on  = [4 12];                       % s      random load: how long it stays on  (min max)
rand_off = [4 12];                       % s      random load: how long it stays off (min max)
du_min   = -30;                          % degCA  lower limit of the ignition retardation (assumption, the manual gives none)
window   = 40;                           % s      width of the sliding time window of the graphs

%% ---------------- run ----------------
live_test(ctrlFile, P_on=P_on, rand_on=rand_on, rand_off=rand_off, du_min=du_min, window=window);
