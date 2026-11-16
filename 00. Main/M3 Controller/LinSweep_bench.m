%% LinSweep_bench.m  -  linearizes Engine_Plant_Norm at several operating points
% Same steps as LinNom_bench.m, repeated for each speed and each nominal ignition retardation.
% Result: LinModels_bench.mat with the struct array M (one entry per operating point).
% Purpose: check that a controller designed at one point stays stable and well damped over 70-200 rad/s.
pd = fullfile(fileparts(fileparts(mfilename('fullpath'))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});   pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models')];   addpath(strjoin(pd(isfolder(pd)), pathsep));   % milestone folders with their functions and models
main;
ident = load('ident_fit_bench.mat').ident;
speeds = [70 100 140 200];                      % rad/s   range required by the manual
retards = [-7 -15];                             % degCA   candidates for delta_u_nom

model = 'Engine_Plant_SteadyState';  model_norm = 'Engine_Plant_Norm';
theta_a_nom = mean(data.meas.T_a.signals.values);
p_a_nom     = mean(data.meas.p_a.signals.values);
theta_m_nom = mean(data.meas.T_m.signals.values);
p_e_nom     = mean(data.meas.p_e.signals.values);
load_system(model);  set_param(model,'StopTime','30');  load_system(model_norm);
for name = {'omega_e','mdot_beta','p_m'}
    ln = find_system(model,'FindAll','on','Type','line','Name',name{1});
    set_param(get_param(ln(1),'SrcPortHandle'),'DataLogging','on');
end

M = struct([]);
fprintf('\n%8s %8s | %9s %9s %7s | %22s | %9s %9s | %s\n','omega','delta_u','u_alpha','p_m','drift','slow mode','gain u_a','gain d_u','max Re(eig)');
for delta_u_nom = retards
    for omega_target = speeds
        u_alpha_nom = fzero(@(u) ssRun(model,u) - omega_target, [0.6 6], optimset('TolX',1e-4));
        [omega_e_nom, mdot_beta_nom, p_m_nom, drift] = ssRun(model, u_alpha_nom);
        Q_nom = mdot_beta_nom/omega_e_nom;
        tau_IPS = 2*pi/omega_e_nom;  tau_seg = 2*pi/(5*omega_e_nom);
        [num1,den1] = pade(tau_IPS,4);  [A1,B1,C1,D1] = tf2ss(num1,den1);
        [num2,den2] = pade(tau_seg,4);  [A2,B2,C2,D2] = tf2ss(num2,den2);
        x0_1 = -A1\B1;   x0_2 = -A2\B2;
        [~,x0] = feval(model_norm,[],[],[],'sizes');
        [A,B,C,D] = linmod(model_norm, x0, [1;1]);
        ev = eig(A);  [~,i] = sort(abs(ev));  g = dcgain(ss(A,B,C,D));
        fprintf('%8.1f %8.1f | %9.4f %9.0f %7.3f | %9.3f %+9.3fj | %9.4f %9.4f | %.3g\n', omega_e_nom, delta_u_nom, u_alpha_nom, p_m_nom, drift, ...
                real(ev(i(1))), imag(ev(i(1))), g(1), g(2), max(real(ev)));
        k = numel(M)+1;
        M(k).A = A; M(k).B = B; M(k).C = C; M(k).D = D; M(k).x0 = x0;
        M(k).omega_e_nom = omega_e_nom; M(k).u_alpha_nom = u_alpha_nom; M(k).delta_u_nom = delta_u_nom;
        M(k).p_m_nom = p_m_nom; M(k).Q_nom = Q_nom; M(k).tau_IPS = tau_IPS; M(k).tau_seg = tau_seg; M(k).drift = drift;
    end
end
Theta_e = ident.Theta_e;  eta_gen = constants.eta_gen;
save(fullfile(fileparts(which('LinSweep_bench')),'LinModels_bench.mat'), 'M','Theta_e','eta_gen','ident')
disp('Saved LinModels_bench.mat')

function [w, mb, pm, drift] = ssRun(model, u)
    in  = Simulink.SimulationInput(model);
    in  = in.setVariable('u_alpha_nom', u);
    out = sim(in);
    s   = out.logsout.get('omega_e').Values;
    w   = s.Data(end);
    mb  = out.logsout.get('mdot_beta').Values.Data(end);
    pm  = out.logsout.get('p_m').Values.Data(end);
    drift = w - interp1(s.Time, s.Data(:), s.Time(end)-5);
end
