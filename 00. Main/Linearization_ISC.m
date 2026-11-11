%% 1. Load parameters and simulate the original plant

options.sim_options = simset('SrcWorkspace', 'current');

run("ISC_Parameters.m");

[tValSim, ~, yvalsim] = sim( ...
    'Engine_Plant', ValData.mdot_in.time, options.sim_options);


%% 2. Determine nominal operating point

p_nominal_calculated     = p_ss.Data(end);
theta_nominal_calculated = theta_ss.Data(end);

% Nominal values of the inputs and output
% (Use the appropriate nominal scaling values from ISC_Parameters.m)

delta_u0 = delta_u_nom_calculated;
u_alpha0 = u_alpha_nom_calculated;
omega_e0 = omega_e_nom_calculated;


%% 3. Normalize the operating point

% States
p0_norm     = p_nominal_calculated / p_nominal;
theta0_norm = theta_nominal_calculated / theta_nominal;

% Inputs
delta_u0_norm = delta_u0 / delta_u_nominal;
u_alpha0_norm = u_alpha0 / u_alpha_nominal;

% Output
omega_e0_norm = omega_e0 / omega_e_nominal;


%% 4. Define nominal input vector

% Two inputs: ignition retardation and throttle opening

u0 = [
    delta_u0_norm;
    u_alpha0_norm
];


%% 5. Define nominal state vector

% Two physical states:
% p_manifold and theta

x0_physical = [
    p0_norm;
    theta0_norm
];

% IMPORTANT:
% Add the 8 initial states of the two Pade approximations.
% These must correspond to their steady-state values
% and must follow the state ordering used by linmod.

% Replace these placeholders with the actual Pade state values:
x0_pade = [
    ... % 4 states of Pade element 1
    ... % 4 states of Pade element 2
];

x0 = [
    x0_physical;
    x0_pade
];


%% 6. Linearize the normalized model

load_system("Engine_Plant_Norm");

[A, B, C, D] = linmod('Engine_Plant_Norm', x0, u0);


%% 7. Check system dimensions

disp('System dimensions:');

fprintf('A: %d x %d\n', size(A,1), size(A,2));
fprintf('B: %d x %d\n', size(B,1), size(B,2));
fprintf('C: %d x %d\n', size(C,1), size(C,2));
fprintf('D: %d x %d\n', size(D,1), size(D,2));

assert(isequal(size(A), [10 10]), 'A must be 10x10');
assert(isequal(size(B), [10 2]),  'B must be 10x2');
assert(isequal(size(C), [1 10]),  'C must be 1x10');
assert(isequal(size(D), [1 2]),   'D must be 1x2');

assert(norm(D, 'fro') < 1e-8, 'Unexpected feedthrough detected');


%% 8. Display system matrices

disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);


%% 9. Save results

save('LinearizationResults.mat', ...
    'A', 'B', 'C', 'D', 'x0', 'u0', ...
    'p0_norm', 'theta0_norm', ...
    'delta_u0_norm', 'u_alpha0_norm', 'omega_e0_norm');
