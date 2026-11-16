
%% This script calculates the extended plant matrices,
% the LQR gain K, observer gain L and complete LQG controller.

%% --- Plant Extension Matrices ---

Ki = 1;

Ae = 0;
Be = [1, 0];
Ce = [Ki; 0];
De = [1, 0;
    0, 1];

%% --- Extended Plant ---

Aext = [A, B*Ce;
    zeros(size(Ae,1),size(A,2)), Ae];

Bext = [B*De;
    Be];

Cext = [C, D*Ce];

Dext = D*De;

%% --- LQR Controller Design ---

r1 = 1;
r2 = 1;

R = [r1, 0;
    0, r2];

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
Dc = zeros(size(K,1),size(Cext,1));

% Complete controller matrices

ISCSAc = [Ae - Be*K, zeros(size(Ae,1),size(Aext,1));
    -L*Ce, Aext - Bext*K - L*Cext];

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
