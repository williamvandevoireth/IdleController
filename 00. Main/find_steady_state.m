% run main first
model = 'Engine_Plant_SteadyState';

% nominal inputs
theta_a_nom = mean(data.meas.T_a.signals.values);
p_a_nom     = mean(data.meas.p_a.signals.values);
theta_m_nom = mean(data.meas.T_m.signals.values);
p_e_nom     = mean(data.meas.p_e.signals.values);

delta_u = 1;
u_alpha = 1;
delta_u_nom = -7;    % degCA, nonzero
u_alpha_nom = 2.5;    % %, tune until omega_e is flat

% steady-state simulation
load_system(model)
set_param(model,'StopTime','30')

% switch on logging for the named lines (no need to click Log Selected Signals)
for name = {'omega_e','mdot_beta','p_m'}
    ln = find_system(model,'FindAll','on','Type','line','Name',name{1});
    if isempty(ln)
        error('No line named "%s" in the model. Name the line (double-click it and type the name).', name{1});
    end
    set_param(get_param(ln(1),'SrcPortHandle'),'DataLogging','on');
end

out = sim(model);
omega_e_nom   = out.logsout.get('omega_e').Values.Data(end);
mdot_beta_nom = out.logsout.get('mdot_beta').Values.Data(end);
p_m_nom       = out.logsout.get('p_m').Values.Data(end);
Q_nom         = mdot_beta_nom/omega_e_nom;

% pade approximation of the two delays
tau_IPS = 2*pi/omega_e_nom;
tau_seg = 2*pi/(5*omega_e_nom);    % half segment, the delay applied to delta_u_nom

[num1,den1] = pade(tau_IPS,4);
[A1,B1,C1,D1] = tf2ss(num1,den1);

[num2,den2] = pade(tau_seg,4);
[A2,B2,C2,D2] = tf2ss(num2,den2);

% normalization
% normalized input is 1 at nominal, so the Pade initial state solves A*x0 c+ B = 0
x0_1 = -A1\B1;
x0_2 = -A2\B2;

disp('Check, both must be 1:')
disp([C1*x0_1 + D1, C2*x0_2 + D2])

disp('Normalization values:')
table(omega_e_nom, u_alpha_nom, delta_u_nom, Q_nom, p_m_nom)

% linearization of the normalized model
model_norm = 'Engine_Plant_Norm';
load_system(model_norm)

[~,x0] = feval(model_norm,[],[],[],'sizes');   % initial states (nominal, if the integrators start at nominal)
[A,B,C,D] = linmod(model_norm, x0, [1;1]);     % normalized nominal input is 1

disp('Sizes (expect 10x10, 10x2, 1x10, 1x2):')
disp([size(A) size(B) size(C) size(D)])
disp(cond(A))
step(ss(A,B,C,D))
