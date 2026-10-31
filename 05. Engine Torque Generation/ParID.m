% script for parameter identification of engine manifold exercise

%clear; close all; clc;

% make sure all simulations use the same options.
    options.sim_options = simset('SrcWorkspace', 'current');

% fmin_data struct
    
    run("ISC_Parameters.m");

    load("IntakeManifoldData/DataIdentification.mat");

 
    whos IdData
    fieldnames(IdData)

% Display measurement values
    figure;
    subplot(2, 1, 1);
    plot(IdData.mdot_in.time, IdData.mdot_in.signals.values,...
        IdData.p.time, IdData.p.signals.values);
        xlabel('Time [s]');
        legend({'p'}, 'Location', 'northwest');


% Find the number of open figures, so a new figure can be created and used
% over and over again. This command is not necessarily needed but it is
% nice to have.
    fh=findall(0,'type','figure');
    options.fig_num = length(fh)+1;
    options.enablePlot = 1;

% Define starting values of the optimization variables.
    par0 = [eta0, eta1, beta0, Theta_e];
    
% Run fminsearch using error function and find optimal parameter values
    % Define function with only one input (parameters to be optimized)
    % with par0, Modelerror runs a simulation and gives back the objective
    % function z = min(y_sim-y_meas)
    errorfcn_fminsearch = @(par0) Modelerror(par0, IdData, options);

% Call fminsearch
    % fminsearch takes objective fcn of Modellerror and computes gradients
    % at minimum, where y_sim = y_meas, the gradient is 0


    lb = [0, -1e-3, 0, 0.01];      % GUESS, CHANGE BEFORE USE
    ub = [1, 0, 20, 1];            % GUESS, CHANGE BEFORE USE
    
    opt_options = optimoptions('fmincon', ...
        'Algorithm','sqp', ...
        'Display','iter', ...
        'MaxIterations',30);
    
    optpar = fmincon(errorfcn_fminsearch, par0, [], [], [], [], ...
        lb, ub, [], opt_options);
% extract optimal parameter values
    eta0 = optpar(1);
    eta1 = optpar(2);
    beta0 = optpar(3);
    Theta_e = optpar(4);

% Load validation data
    load("DataValidation.mat");

% Run a simulation with the identified optimal parameter values
    data.mdot_in = ValData.mdot_in;
    data.p = ValData.p;

    [tValSim, ~,yvalsim] = sim('NonlinearModel', ValData.mdot_in.time, options.sim_options);


% Graphically represent the quality of the optimization routine
    figure;
    subplot(2,1,1);
    plot(tValSim, ValData.mdot_in.signals.values, ...
         tValSim, ValData.p.signals.values);
        xlabel('Time [s]');
        ylabel('Inputs');
        legend('mdot_{in}','p');
    subplot(2,1,2);
    plot(tValSim, ValData.p.signals.values, ...
        tValSim, yvalsim);
        xlabel('Time [s]');
        ylabel('Outputs');
        legend('Measured p','Modelled p')
   
    