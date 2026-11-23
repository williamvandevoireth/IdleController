
%% This script calculates the extended plant matrices,
% the LQR gain K, observer gain L and complete LQG controller.

run("ISC_Parameters.m") 


%% --- Plant Extension ---
Ki = 1;

% Extension Matrices 
Ae = 0;
Be = [1, 0];
Ce = [Ki; 0];
De = [1, 0; 0, 1];

% Extended Plant
Aext = [A, B*Ce; zeros(size(Ae,1),size(A,2)), Ae];
Bext = [B*De; Be];
Cext = [C, D*Ce];
Dext = D*De;

%% --- LQR Controller Design ---
r1 = 1;
r2 = 1;
R = [r1, 0; 0, r2];

Q = Cext' * Cext;
K = lqr(Aext, Bext, Q, R);

disp('LQR Gain K:')
disp(K)

%% --- State Observer Design ---
q = 1;

L = lqr(Aext', Cext', Bext*Bext', q)';

disp('Observer Gain L:')
disp(L)

%% --- Observer State Space Matrices ---
Aobs = Aext - L*Cext;
Bobs = [Bext - L*Dext, L];
Cobs = Cext;
Dobs = [Dext, zeros(size(Dext,1),size(L,2))];

%% --- Complete LQG Controller ---

% Controller feedthrough matrix
Dc = zeros(size(De,2),size(Cext,1));

% Complete controller matrices
ISCSAc = [Ae, -Be*K;
    zeros(size(Aext,1),size(Ae,1)), ...
    Aext - Bext*K - L*Cext];

ISCSBc = [Be*Dc;
    -L];

ISCSCc = [Ce, -De*K];

ISCSDc = De*Dc;

disp('Complete LQG Controller Matrices:')

disp('ISCSAc:')
disp(ISCSAc)

disp('ISCSBc:')
disp(ISCSBc)

disp('ISCSCc:')
disp(ISCSCc)

disp('ISCSDc:')
disp(ISCSDc)

%% --- set up simulation
% Define Input Signals

% Simulation time
t = (0:0.01:10)';

% Example input signals
u1 = sin(t);
u2 = zeros(size(t));

% Combine inputs into one signal
inputData = [u1, u2];

% Create timeseries
inputSignal = timeseries(inputData, t);
%% --- run simulation
model = 'Observer_Test_Leon';

% Modell laden
load_system(model);

% Simulationskonfiguration
simIn = Simulink.SimulationInput(model);

% Eingangssignale übergeben
simIn = simIn.setExternalInput(inputSignal);

% Simulationsdauer festlegen
simIn = simIn.setModelParameter('StopTime', '10');

% Simulation starten
simOut = sim(simIn);

%% --- Review simulation results ---

% Read error signal from simulation
errorSignal = simOut.errorSignal;

% Extract time and error values
t_error = errorSignal.Time;
e = errorSignal.Data;

% Plot errors
figure;

subplot(2,1,1);
plot(t_error, e(:,1));
grid on;
ylabel('Error e_1');

subplot(2,1,2);
plot(t_error, e(:,2));
grid on;
xlabel('Time [s]');
ylabel('Error e_2');