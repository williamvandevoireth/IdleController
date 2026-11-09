

options.sim_options = simset('SrcWorkspace', 'current');
run("ISC_Parameters.m");
[tValSim, ~,yvalsim] = sim('Engine_Plant', ValData.mdot_in.time, options.sim_options);


p_nominal_calculated = p_ss.Data(end);
theta_nominal_calculated = theta_ss.Data(end);

% Normierter Druck = 1
delta_u_norm = delta_u_nom_calculated / delta_u_nominal;

% Normierte Temperatur  = 1
u_alpha_norm = u_alpha_nom_nominal_calculated / theta_nominal;

% Normierte Drehzahl = 1
omega_e_nom = omega_e_nom_calculated / omega_e_nominal;

% Normierter Eingang
u0 = 1;

% Zustandsvektor
x0 = [p0_norm; theta0_norm];
% mdot_ou_ss has to be the same as mdot_in

load_system("Engine_Plant_Norm");

[A, B, C, D] = linmod('Engine_Plant_Norm', x0, u0);

% Matrizen für die weitere Verwendung speichern
save('LinearizationResults.mat', ...
    'A', 'B', 'C', 'D', 'x0', 'u0');

% Kontrolle
disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);


