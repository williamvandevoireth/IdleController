%% LinNom_bench.m  -  LinNom.m with the parameters identified on the bench + diagnostics
% Differences from LinNom.m:
%   - loads ident_fit_bench.mat (LinNom.m used the initial guesses of ISC_Parameters.m)
%   - finds u_alpha_nom automatically so that the steady-state speed is omega_target
%   - checks that the nominal point is an equilibrium of Engine_Plant_Norm
%   - prints eigenvalues, singular values and condition numbers, saves the matrices
pd = fullfile(fileparts(fileparts(mfilename('fullpath'))), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});   pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models')];   addpath(strjoin(pd(isfolder(pd)), pathsep));   % milestone folders with their functions and models
main;
ident = load('ident_fit_bench.mat').ident;      % parameters identified on studentId_0002/0003
omega_target = 140;                             % rad/s  linearization speed

model = 'Engine_Plant_SteadyState';

% nominal inputs (measured means)
theta_a_nom = mean(data.meas.T_a.signals.values);
p_a_nom     = mean(data.meas.p_a.signals.values);
theta_m_nom = mean(data.meas.T_m.signals.values);
p_e_nom     = mean(data.meas.p_e.signals.values);
delta_u_nom = -7;                               % degCA, nonzero so that the ignition can act both ways

%% 1. Steady state: find u_alpha_nom that gives omega_target
load_system(model)
set_param(model,'StopTime','30')
for name = {'omega_e','mdot_beta','p_m'}
    ln = find_system(model,'FindAll','on','Type','line','Name',name{1});
    if isempty(ln), error('No line named "%s" in %s.', name{1}, model); end
    set_param(get_param(ln(1),'SrcPortHandle'),'DataLogging','on');
end
u_alpha_nom = fzero(@(u) ssRun(model,u) - omega_target, [1.2 4.5], optimset('TolX',1e-4));
assignin('base','u_alpha_nom',u_alpha_nom);
[omega_e_nom, mdot_beta_nom, p_m_nom, drift] = ssRun(model, u_alpha_nom);
Q_nom = mdot_beta_nom/omega_e_nom;
fprintf('Steady state: u_alpha_nom = %.4f %%, omega_e = %.2f rad/s, p_m = %.0f Pa, drift over the last 5 s = %.3f rad/s (must be about 0)\n', ...
        u_alpha_nom, omega_e_nom, p_m_nom, drift);

%% 2. Pade approximation of the two delays
tau_IPS = 2*pi/omega_e_nom;
tau_seg = 2*pi/(5*omega_e_nom);
[num1,den1] = pade(tau_IPS,4);  [A1,B1,C1,D1] = tf2ss(num1,den1);
[num2,den2] = pade(tau_seg,4);  [A2,B2,C2,D2] = tf2ss(num2,den2);
x0_1 = -A1\B1;   x0_2 = -A2\B2;
fprintf('Pade output at equilibrium (both must be 1): %.6f  %.6f\n', C1*x0_1+D1, C2*x0_2+D2);
fprintf('cond(A1) = %.3g   cond(A2) = %.3g   (companion form of tf2ss)\n', cond(A1), cond(A2));

%% 3. Is the nominal point an equilibrium of Engine_Plant_Norm ?
model_norm = 'Engine_Plant_Norm';
load_system(model_norm)
[~,x0] = feval(model_norm,[],[],[],'sizes');
try
    feval(model_norm,[],[],[],'compile');
    feval(model_norm, 0, x0, [1;1], 'outputs');          % updates the block signals, required before 'derivs'
    dx = feval(model_norm, 0, x0, [1;1], 'derivs');
    feval(model_norm,[],[],[],'term');
    fprintf('\nState derivatives at the nominal point:\n   state           x0          dx/dt\n');
    fprintf('   %2d   %14.6g %14.6g\n', [(1:numel(dx)); x0(:)'; dx(:)']);
    fprintf('Physical states: d(omega_e)/dt = %.4g rad/s^2, d(p_m)/dt = %.4g Pa/s (both must be about 0)\n', dx(1), dx(2));
catch err
    try feval(model_norm,[],[],[],'term'); catch, end
    fprintf('Equilibrium check skipped: %s\n', err.message);
end

%% 4. Linearization and diagnostics
[A,B,C,D] = linmod(model_norm, x0, [1;1]);
fprintf('\nSizes (expect 10x10, 10x2, 1x10, 1x2): %s\n', mat2str([size(A) size(B) size(C) size(D)]));
ev = eig(A);  sv = svd(A);
fprintf('Eigenvalues of A (all real parts must be negative):\n');  disp(ev)
fprintf('largest real part = %.4g\n', max(real(ev)));
fprintf('singular values: max = %.3g, min = %.3g  ->  cond(A) = %.3g\n', max(sv), min(sv), cond(A));
[~,Ab] = balance(A);
fprintf('cond(A) after diagonal balancing of the states = %.3g\n', cond(Ab));
sys = ss(A,B,C,D);
fprintf('DC gain  omega_e/u_alpha = %.4g,  omega_e/delta_u = %.4g  (normalized; expected signs: + for the throttle, - for the ignition: more retard gives less torque)\n', dcgain(sys));
fprintf('norm of D = %.3g (must be 0: no feed-through)\n', norm(D));
figure(5); clf; step(sys, 5); grid on
save(fullfile(fileparts(which('LinNom_bench')),'LinModel_bench.mat'), 'A','B','C','D','x0','omega_e_nom','u_alpha_nom','delta_u_nom', ...
     'Q_nom','p_m_nom','tau_IPS','tau_seg','ident','theta_a_nom','p_a_nom','theta_m_nom','p_e_nom')

%% Local function: steady-state simulation for one throttle opening
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
