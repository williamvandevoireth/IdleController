% ISC_Parameters.m
% One variable per line. The comment after each variable gives its unit and the block(s) that use it.
% Rules for the whole group:
%  - Never define the same variable in another file (the last definition silently wins).
%  - Never rename a variable without renaming it in every block that reads it.
%  - While a line carries the tag TEMP, its value is an initial guess from the exercise manual,
%    to be replaced by the identified value (milestone 2).
% Source: Exercise Instructions Manual, section 2.2.3 (parameter table of each block).

%% --- Simulation settings
Ts = 1e-3;              %s            fixed solver step (ode1) and sampling time, data recorded at 1 kHz (manual 2.2.5)

%% --- Known constants
R = 287;                %J/(kg*K)     specific gas constant of air (Throttle Air Mass Flow, Intake Manifold)
Vd = 2.48e-3;           %m^3          displacement volume (Engine Air Mass Flow, Engine Torque Generation)
Vc = 2.48e-4;           %m^3          compression volume (Engine Air Mass Flow)
sigma0 = 14.7;          %-            stoichiometric air/fuel ratio, table value (the text of the manual says 14.67) (Engine Air Mass Flow)
kappa = 1.35;           %-            exponent of the volumetric efficiency, pressure part (Engine Air Mass Flow)
lambda = 1;             %-            air/fuel ratio, kept at 1 (Engine Air Mass Flow, Engine Torque Generation)
Hl = 42.5e6;            %J/kg         lower heating value of gasoline (Engine Torque Generation)
k_zeta = 2.3345e-4;      %1/(degCA)^2  ignition efficiency coefficient, given (Engine Torque Generation)
eta_gen = 0.7;          %-            generator efficiency (Load Torque, valid for 70-200 rad/s)

%% --- Parameters to identify: initial guesses (TEMP)
alpha0 = 3e-6;          %m^2          throttle area offset, TEMP (Throttle Area)
alpha1 = 6e-6;          %m^2          throttle area slope per % opening, TEMP (Throttle Area)
Vm = 7e-3;              %m^3          intake manifold volume, TEMP (Intake Manifold)
gamma0 = 0.6;           %-            volumetric efficiency, speed part, offset, TEMP (Engine Air Mass Flow)
gamma1 = 0.002;         %s/rad        volumetric efficiency, speed part, slope, TEMP (Engine Air Mass Flow)
eta0 = 0.3;             %-            Willans efficiency offset, TEMP (Engine Torque Generation)
eta1 = -3e-4;           %s/rad        Willans efficiency slope, TEMP (Engine Torque Generation)
beta0 = 7;              %Nm           friction torque, TEMP (Engine Torque Generation)
Theta_e = 0.2;            %kg*m^2       engine inertia, TEMP (Engine Inertia)

%% --- Initial conditions of the dynamic blocks (PROPOSAL, to be agreed by the group)
pm_init = 3e4;          %Pa           intake manifold pressure (Intake Manifold)
w_init = 100;           %rad/s        engine speed (Engine Inertia)
q_init = 3e-5;          %kg/rad       initial output of the IPS delay, mdot_beta/omega_e (Engine Torque Generation)
du_init = 0;            %degCA        initial output of the ignition delay (Engine Torque Generation)
theta_a = 298;          %K            ambient temperature
p_a = 100000;           %Pa           ambient pressure
theta_m = theta_a;      %K            NOT FIX, ONLY GUESS
p_e = 99800;            %Pa           NOT FIX, ONLY GUESS
theta_e = theta_a;      %K            NOT FIX, ONLY GUESS
omega_e0 = 700;         %rad/s        NOT FIX, ONLY GUESS
p_m0 = 100000;          %Pa           NOT FIX, ONLY GUESS

%% --- Speed limits (manual 2.3.7: avoid physically meaningless speeds during identification)
w_min = 20;             %rad/s        lower saturation of the engine speed
w_max = 500;            %rad/s        upper saturation of the engine speed

%% --- Controller design (milestone 3): add the variables here when they exist

u_l = 0;                % on/off signal 1 or 0
P_l = 100;              % kW load power

