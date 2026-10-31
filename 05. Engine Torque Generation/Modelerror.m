function V = Modelerror(par, data, options)

% This is the so called 'error function' (modellguete.m) that is used in 
% combination with fminsearch.

% Set global optimization variables
%     global tau beta
% bring parameter into current workspace
    run("ISC_Parameters.m");
% Assign new parameter values (used in Simulink model SystemModel.slx)
    
    % Starting values directly from ISC_Parameters.m
    eta0 = par(1);
    eta1 = par(2);
    beta0 = par(3);
    Theta_e = par(4);
    
    % Bounds (example values, adjust if necessary)

    disp(['eta0 = ', num2str(eta0)])
    disp(['eta1 = ', num2str(eta1)])
    disp(['beta0 = ', num2str(beta0)])
    disp(['Theta_e = ', num2str(Theta_e)])


% Run simulation to determine the quality of the parameter values  

    [tsim, ~, ysim] = sim('NonlinearModel', data.mdot_in.time, options.sim_options);
    disp(['p_start = ', num2str(ysim(1))])
    disp(['p_end   = ', num2str(ysim(end))])
    disp(['mean mdot_in  = ', num2str(mean(data.mdot_in.signals.values))])
    disp(['mean p_sim = ', num2str(mean(ysim))])
    %out = sim('NonlinearModel', data.mdot_in.time, options.sim_options);
    %out.who
% Calculate objective function
    disp(['Messbereich: ', num2str(min(data.p.signals.values)), ' ... ', num2str(max(data.p.signals.values))])
    disp(['Simbereich:  ', num2str(min(ysim)), ' ... ', num2str(max(ysim))])
    V = sum((data.p.signals.values-ysim).^2);

% plot curves
if options.enablePlot
    figure(options.fig_num);
    plot(data.p.time, data.p.signals.values, 'b'); hold on; grid on;
    plot(tsim, ysim, '-r' ); hold off;
    xlabel('Time [s]');
    ylabel('Model Ooutput [-]');
	legend({'Measurements','Simulation'},'Location','NorthEast')
    set(gca,'XLim',[data.p.time(1) data.p.time(end)]);
    set(gca,'YLim',[min(data.p.signals.values)-1/10*mean(data.p.signals.values) max(data.p.signals.values)+1/10*mean(data.p.signals.values)]);
    drawnow;
end