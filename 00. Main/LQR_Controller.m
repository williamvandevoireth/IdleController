%% This script calculates the extended plant matrices, the LQR gain K and the observer gain L based on Ki, r1, r2 and q
%% Plant Extension Matrices

Ki = 1;

Ae = 0;
Be = [1, 0];
Ce = [Ki; 0];
De = [1, 0;
      0, 1];

%% --- Extended Plant ---

Aext = [A, B*Ce; zeros(size(Ae,1),size(A,2)), Ae];
Bext = [B*De; Be];
Cext = [C, D*Ce];
Dext = D*De;

%% --- LQR Controller Design ---

    % Weighting matrix R
    r1 = 1;
    r2 = 1;

    R = [r1, 0;
    0, r2];

    % Weighting matrix Q
    Q = Cext' * Cext;

    % Calculate state feedback gain
    K = lqr(Aext, Bext, Q, R);
    disp('K:')
    disp(K)

%% --- State Observer Design ---

    q = 1;

    L = lqr(Aext', Cext', Bext*Bext', q)';

    disp('Observer Gain L:')
    disp(L)
