# IdleController

Team project of the ETH course *Engine Systems* (151-0567-00, Fall 2026, Prof. C. Onder; exercise supervisors G. Fieni, J. Wüthrich):
**idle speed control (ISCS) of a 2.5 l, 5-cylinder SI engine**, in MATLAB/Simulink (R2026b). Team of 3-5. The exercise gives up to 0.25 grade points
(only if it improves the final grade); the grade itself is a 30 min oral exam that also covers the exercise. Manual: `2026_ExerciseInstructionsManual.pdf` (not in this repo; page numbers = printed).

*Status 2026-11-16:* M1 and M2 done (model error still large), M3 linearization done and one LQGI controller designed, **no competition file and no closed-loop bench in the repo yet**.

## Milestones, deadlines, status

| # | Milestone | Deadline | State |
|---|-----------|----------|-------|
| 1 | Nonlinear plant model in Simulink | Oct 25 | done: `M1 Model/Engine_Plant.slx` |
| 2 | Identification + validation on data | Nov 8 | done as scripts, results saved; error ~10 rad/s (see Results) |
| 3 | Normalization, linearization, controller working in simulation | Nov 29 | linearization done; LQGI design exists (`M3 Controller/William/`), a second script `00. Main/LQR_Controller.m` started; closed-loop result not good enough at low speed |
| 4 | Controller on the test bench, competition | Dec 10 (class 4 said Dec 13; competition Dec 11 per fact sheet) | not started: needs `controller1.mat` / `controller2.mat` |

Test bench (ML E44, dSpace ECU): model-validation session week of Nov 2 (done, our data = `studentId_*`), controller sessions weeks of Nov 30 and Dec 7. Two controllers per team; lowest cost J wins (pizza voucher).

## Repository layout

```
IdleController/
├── ISCSData/                 recordings (see Data)
├── 00. Main/                 working folders, one per milestone
│   ├── M1 Model/             Engine_Plant.slx, main.m, ISC_Parameters.m
│   ├── M2 Identification/    ParID.m, ParID_bench.m, costVm.m, costTorque.m, Validation.m, ident_fit.mat, ident_fit_bench.mat
│   ├── M3 Controller/        Engine_Plant_Norm.slx, Engine_Plant_SteadyState.slx, LinNom.m, LinNom_bench.m, LinSweep_bench.m,
│   │   │                     Linearization_ISC.m (unfinished draft, unused), LinModel_bench.mat, LinModels_bench.mat
│   │   └── William/          personal folder: LQGI_run_William.m, Controller_William.slx
│   ├── M4 Competition/       Engine_Plant_CL.slx (Engine_Plant + omega_e outport, for closed-loop tests)
│   └── LQR_Controller.m      teammate's LQR/observer script (needs A,B,C,D in workspace; placeholder weights r1=r2=q=1)
└── 01. Throttle Area ... 07. Load Torque/    first per-block models (ThrottleArea, ThrottleAMF, IntakeManifold, AMF_model, Engine_Torque(+ParID, Modelerror), EngineInertia, LoadTorque), kept for reference
```
Convention: shared folders M1-M3 have one owner; personal work goes into a personal subfolder; one person edits a given `.slx` at a time. `.slxc`, `slprj/` are git-ignored.

## How to run
1. Current folder = repo; scripts add the milestone folders to the path themselves (`addpath(...)` line): `ParID*.m`, `Validation.m`, `LinNom_bench.m`, `LinSweep_bench.m`, `LQGI_run_William.m`, `main.m`.
   `LinNom.m`, `LQR_Controller.m`, `Linearization_ISC.m` have no path line: run `main` first.
2. `main` builds the workspace (**it starts with `clear`**): structs `constants, ident (initial guesses), init, limits, simcfg, data`. `data_name` in `main.m` picks the recording (default `studentId_0003`). `data.meas.T_a.signals.values` etc. are read by the From-Workspace blocks of the plant.
3. Order: `ParID_bench` (~20 min, writes `ident_fit_bench.mat`) -> `LinNom_bench` (writes `LinModel_bench.mat`) -> `LinSweep_bench` (writes `LinModels_bench.mat`) -> `LQGI_run_William` (design; no .mat is saved).
   `Validation.m` and `ParID.m` are the earlier version (fit `studentId_0002`, `ident_fit.mat`).

## The plant (manual 2.2.3, p. 15-24)
Causality chain (Fig. 2.5): `u_alpha -> Throttle Area -> Throttle Air Mass Flow -> Intake Manifold -> Engine Air Mass Flow -> Engine Torque Generation -> Engine Inertia -> omega_e`
(+ `Load Torque` as disturbance, + `p_m`, `p_e`, `theta_m` feedback). Inputs: `u_alpha` [0..100 %], `delta_u` (ignition retardation from MBT, degCA, **<= 0**; positive values are replaced by 0 on the bench).
States: `omega_e`, `p_m` (+ 2 delays). Output: `omega_e`. Disturbance: `u_l` (on/off), `P_l` (~1 kW water boiler, ~1.4 kW mechanical). Speed range of interest 70-200 rad/s. Solver: fixed-step ode1, 1 ms. Keep exactly two variable transport delays.

| Block | Equation |
|-------|----------|
| Throttle area | `A = alpha0 + alpha1*u_alpha` |
| Throttle flow | `mdot_a = A*p_a/sqrt(R*theta_a)*(1/sqrt2)` if `p_m/p_a < 1/2`; else `A*p_a/sqrt(R*theta_a)*sqrt(2*(p_m/p_a)*(1-p_m/p_a))` |
| Intake manifold | `dp_m/dt = R*theta_m/Vm * (mdot_a - mdot_b)` (isothermal, no kappa) |
| Engine air flow | `mdot_e = 1/2 * p_m/(R*theta_m) * lambda_l * Vd*omega_e/(2*pi)`; `mdot_b = mdot_e/(1+1/(lambda*sigma0))`; `lambda_l = lambda_lp*lambda_lw`; `lambda_lp = (Vc+Vd)/Vd - Vc/Vd*(p_e/p_m)^(1/kappa)`; `lambda_lw = gamma0 + gamma1*omega_e` |
| Torque (Willans) | `T_e = (e*pm_phi - pme0f - pme0g)*Vd/(4*pi)`, `e*pm_phi = (eta0+eta1*omega_e)*e_zeta*Hl/Vd * 4*pi/(lambda*sigma0) * mdot_b(t-tau_IPS)/omega_e(t-tau_IPS)` (as in manual), `e_zeta = 1 - k_zeta*du(t-tau_seg/2)^2`, `tau_IPS = 2*pi/omega_e`, `tau_seg = 4*pi/(5*omega_e)`, `pme0f = 4*pi*beta0/Vd`, `pme0g = p_e - p_m` |
| Inertia | `d omega_e/dt = (T_e - T_l)/Theta_e` |
| Load torque | `T_l = P_l/(omega_e*eta_gen)`; simulation only, never part of the controller or the linear model |

Known constants: `R=287, Vd=2.48e-3, Vc=2.48e-4, kappa=1.35, sigma0=14.7 (table; the text says 14.67), Hl=42.5e6, k_zeta=2.3345e-4 1/degCA^2, eta_gen=0.7, lambda=1`.
To identify (initial guess): `alpha0 (3e-6), alpha1 (6e-6), Vm (7e-3), gamma0 (0.6), gamma1 (0.002), eta0 (0.3), eta1 (-3e-4), beta0 (7 Nm), Theta_e (0.2)`.
Models in the repo: `Engine_Plant.slx` (physical units; replays `T_a, p_a, T_m, p_e, P_l, u_l` from `data.meas`; `simcfg.omega_meas=0` -> own simulated speed; MATLAB Function blocks read constants only if declared as **Parameter data**).
`Engine_Plant_Norm.slx`: normalized (Fig. 2.11), delays as 4th-order Padé State-Space blocks (frozen at the linearization speed), 10 states, no load; needs `ident, omega_e_nom, u_alpha_nom, delta_u_nom, Q_nom, p_m_nom, theta_a_nom, p_a_nom, theta_m_nom, p_e_nom, A1..D1, x0_1, A2..D2, x0_2` (set by `LinNom_bench.m`).
`Controller_William.slx`: LQG block + integrator extension in series (Fig. 2.16) and a discrete 12-state block; matrices live in the model workspace.

## Procedure per milestone (manual) and class hints
**M2 identification (p. 25-30), modular, linear least squares whenever possible:** (1) `alpha0, alpha1`: `A = mdot_a*sqrt(2*R*theta_a)/p_a` vs `u_alpha`, quasistatic data, needs `p_m/p_a <= 0.5`;
(2) `gamma0, gamma1`: `lambda_lw = 4*pi*R/Vd * theta_m*mdot_a/(p_m*omega_e*lambda_lp) * (1+1/(lambda*sigma0))`, quasistatic data; (3) `Vm` with `fminsearch` on `sum (p_m - p_m_model)^2`, throttle flow from the model, measured `omega_e` fed in;
(4) `eta0, eta1, beta0, Theta_e` with a nonlinear fit (`fminsearch`/`fmincon` with bounds, e.g. `Theta_e > 0`), `omega_e` saturated to 20-500 rad/s; (5) validate on data **not used** for the fit. `eta0` partly compensates for `beta0` (not necessarily physical).
Class hints: use simulated `p_m` (feedback) in the identification, not the measured one, otherwise the optimizer diverges; initialize integrators from the data (`p(1)` or `mean(p(1:10))`); use fixed step sizes, no variable step; recommended order with the course data: fit on `dynamic_0003`, validate on `dynamic_0002` (the other way round failed in earlier years); normalize data before least squares (`cond(M'M)=cond(M)^2`); good start values (`Vm` about 3 x `Vd`).
**M3 normalization/linearization (p. 31-32):** pick nominal inputs, run to steady state (get states, outputs, delay times); two delay blocks only; replace each by a 4th-order Padé (`pade`, State-Space block, initial state `x0 = -A\B`, normalize input and output but **not** the Padé states);
build the normalized model (every input/state/output divided by its nominal value; delta values via `u -> (u-u0)/u0`); `linmod(model, x0, [1;1])` on the **normalized** model; expect 10 states, 2 inputs, 1 output, D = 0; check step response and `cond`.
Class hints: normalize first, linearize afterwards; linearization point must be a true equilibrium; all matrix entries should be around 1; to compare with the nonlinear model denormalize and add the nominal point back (the `1`s in Fig. 1.3).
**Specifications (p. 32-33):** reference 70-200 rad/s; random load steps; zero steady-state error for constant load and reference change; `omega_e >= 50 rad/s` (stall); reasonable overshoot; minimum return difference `mu = min|1+L(jw)| >= 0.5`; crossover of `L=P*C` preferably <= 4 rad/s (noise); `Ts < pi/(5*wc)`.
**LQGI (p. 33-36):** extension `Ae=0, Be=[1 0], Ce=[K1;0], De=I` on the throttle channel; extended plant `A=[As Bs*Ce; 0 Ae], B=[Bs*De; Be], C=[Cs Ds*Ce]`; `K=lqr(A,B,C'*C,diag(r1,r2))`; `L=lqr(A',C',B*B',q)'` (q small = fast observer);
controller `ISCS_Ac=[Ae -Be*K; 0 A-B*K-L*C]`, `ISCS_Bc=[Be*Dc; -L]`, `ISCS_Cc=[Ce -De*K]`, `ISCS_Dc=De*Dc`, `Dc=0`; input `e = ref - omega`; discretize with Tustin (`c2d`), Ts = 1 ms. Guideline: observer poles about 3x faster than `A-BK`. Add band-limited white noise in simulations. The ignition should return to its nominal value after the throttle has adapted (Fig. 2.3).
**M4 (p. 39):** files `controller1.mat`, `controller2.mat` with `ISCS_Ad (12x12), ISCS_Bd (12x1), ISCS_Cd (2x12), ISCS_Dd (2x1), ISCS_Tu = diag(u_alpha_nom, delta_u_nom), ISCS_Ty = omega_e_nom`.
Cost `J = int( 0.2*kmax^2.5*|omega_e - omega_ref| + |d u_alpha/dt| ) dt`, `kmax = 1/(1 - k_zeta*delta_u_nom^2)` (`u_alpha` in %, 0..100). A large `|delta_u_nom|` costs fuel (kmax) but gives the ignition more authority: a design trade-off.

## Data (`ISCSData/`, 1 kHz, struct `meas`, each signal `meas.<name>.time` / `.signals.values`)
Signals: `u_l` (0/1 load), `P_l` [W], `p_a, p_e, p_m` [Pa], `T_a, T_m` [K], `m_dot_alpha` [kg/s], `omega_e` [rad/s], `u_alpha` [%], `du_ign` [degCA], `omega_e_desired` [rad/s], `U_Bat, I_Gen, lambda, cont_out` (not needed for identification).
`quasistatic_0001` (200 s, slow throttle ramp, static ID), `dynamic_0002` (80 s, load 26 %), `dynamic_0003` (70 s, load 29 %) are course data;
`studentId_0002` (120 s, throttle staircase, no load, 72-258 rad/s) and `studentId_0003` (120 s, load steps ~1.2 kW 40 % of the time, 67-260 rad/s) are **our** bench recordings of 2026-11-02.
**`du_ign` is 0 in all five files**: the ignition-to-torque path (k_zeta) is not validated by any data.

## Preliminary exercise (intake manifold, not in this repo)
Done. Results: `Vm = 0.00923 m^3`, `lambda_lw = 1.0379`, eigenvalues of A -2.474 and -4.093 1/s, validation RMSE about 850 Pa. Its scripts (`Main, Parameters, ParID, Modelerror, Linearization`, `ParIdDemo/` with `fminsearch`) are the template for M2/M3.

## Results so far
- **Identification (`ident_fit_bench.mat`):** `alpha0=1.776e-6, alpha1=5.801e-6, Vm=7.5e-3 (almost the initial guess), gamma0=0.5371, gamma1=0.0018, eta0=0.4282, eta1=-5.096e-4, beta0=11.13, Theta_e=0.1848`.
- **Model quality (simulated vs measured `omega_e`, p_m):** `Validation.m` with `ident_fit.mat`: `studentId_0002` 3.51 rad/s / 1262 Pa, `studentId_0003` 8.10 rad/s / 1640 Pa. Bench fit on `studentId_0003`: 9.7 rad/s (fit half), 9.6 rad/s / 2063 Pa (validation half).
  Shape and timing agree; the error is a speed-dependent offset (sim minus meas: +4 rad/s at 77, +3 at 133, -3 at 171, -7.5 at 201, -15 at 242 rad/s); `p_m` at 1 % throttle 2200 Pa too low; the model oscillates at low speed where the data does not. The load is not the main error source.
- **Linearization (`LinModel_bench.mat`):** nominal 140 rad/s, `delta_u_nom=-7`, `u_alpha_nom=2.367 %`, `p_m=19785 Pa`; 10x10, stable (max real part -1.24; slow pair -1.24 +/- 1.48j), plant acts like 2nd order (Hankel values), `cond(A)=1.55e19` (Padé companion form) -> 1.9e4 after `balance`: **always balance before designing**.
  DC gain 0.73 (u_alpha), **-0.017 (delta_u)**: at -7 degCA the ignition has almost no authority (more at -15, at higher kmax cost). Sweep (`LinModels_bench.mat`, 70/100/140/200 rad/s, delta_u -7/-15): stable everywhere, slowest real part -0.36 / -0.72 / -1.24 / -2.14; the 70 rad/s points drift ~1.2 rad/s (not settled).
- **LQGI design (William):** `r1=10, r2=10, qw=100 (Q=qw*C'*C), qobs=1e-6, K1=1`; linear model: `min|1+L|=0.864`, crossover 3.03 rad/s, stable, 0 % overshoot; matrices have the competition sizes; the "3x faster observer" guideline is not reachable (both limited by the plant's slow pole ~-1).
  Normalization of controller input/output as deviations from nominal is **inferred** (Fig. 2.15, p. 39), not stated.
- **Closed loop on `Engine_Plant_CL` (recorded load replayed, 1 sample delay; bench and `controller1.mat` no longer in the repo):** works for references 110-180 rad/s (dip to ~88 rad/s at the 1.2 kW step, recovers); **stalls for references <= 90 rad/s with load** (speed 20-210 rad/s oscillation); 100 rad/s survives (min 62.6); throttle spikes at reference steps. Likely (inferred): design point 140 rad/s vs slower plant and larger load torque at low speed, throttle at 0 %, no anti-windup. J = 537 for the test profile 140/110/180/90/140.

## Open items
1. Make the controller work over 70-200 rad/s with random loads (design at lower speed, detuning, gain scheduling, anti-windup, other `delta_u_nom`, maybe larger `r1`).
2. Rebuild a closed-loop bench on `Engine_Plant_CL.slx` (recorded load or random load steps, measurement noise, cost J) in `M4 Competition`; then save `controller1.mat` / `controller2.mat`.
3. Reduce the identification error (restrict fit to 70-200 rad/s; Vm did not move). Ask the supervisors for a recording with ignition steps, or accept the manual's formula.
4. Manual p. 23 bonus questions unanswered/unconfirmed: which part accounts for the auxiliary power (probably `beta0`/`eta0`); why are the Padé states not normalized.
5. `resources/project/` files show as staged deletions in `git status`; `IdleController.prj` is an empty MATLAB project.

## Course facts (class facts sheet, exercise classes 1-6)
- Textbook: Guzzella/Onder, *Introduction to Modeling and Control of Internal Combustion Engine Systems* (2nd ed. 2010, with corrigendum). Documents are on Moodle and `idsc.ethz.ch/education/lectures/engine-systems.html`. Lecture Mon 8:15, exercise class Mon 12:15 (ML F 36). Recap on modeling/control was held Oct 7.
- Theory sheets 1-6 are solved **individually** (solutions on Moodle); the ISCS exercise is group work (>= 3 people, at least 2 who know MATLAB/Simulink and 2 who know control systems).
- Exam: oral, 30 min, English or Swiss German. Scope: theory sheets, engine components and operating principles, modeling equations and their assumptions (reproduce/derive them), main engine control tasks, back-of-envelope numbers (atmospheric pressure, lower heating value, fuel properties). **The ISCS exercise is relevant for the exam.**
- Test-bench sessions are booked per group on Moodle; the competition disturbance (load) is unknown in advance, so the controller must be robust, not tuned to a known profile.

## Modeling method (class 1, reservoir approach) and Willans model (class 5)
Model types: black box (experiments only), white box (physics + known parameters), gray box (physics + identified parameters; used here: equations transferable, ParID changes per application).
Procedure: (1) define boundaries/inputs/outputs; (2) pick reservoirs and their level variables (kinetic energy -> speed; mass/thermal -> pressure, temperature); fast dynamics -> algebraic, relevant dynamics -> differential equation, slow dynamics -> constant;
(3) draw the causality diagram (same structure as the Simulink model); (4) conservation laws `d/dt content = sum inflows - sum outflows`; (5) algebraic flow equations as functions of level variables; (6) identify parameters (static: least squares, e.g. `c_d`, `lambda`; dynamic: `fminsearch`, e.g. `Vm`, `Theta`); (7) validate on other experiments.
Willans approximation: `pme = e*pmphi - pme0`, with `e` = thermodynamic conversion efficiency and `pme0` = friction + gas exchange losses; affine in fuel energy per cycle. Typical values: `e ~ 0.5` at full load, `pme0 ~ 1 bar` for a gasoline engine at idle (closed throttle -> pumping losses), `~ 0.1 bar` at full load. Zero torque while injecting fuel = idling (only losses are covered); efficiency rises with load.
Here: constant friction (`beta0`), `e = eta0 + eta1*omega_e` times the ignition efficiency `e_zeta`.

## Simulink / MATLAB practices from the classes
- Fixed-step solver with an explicit step (`simset('SrcWorkspace','current','Solver','ode1','FixedStep',1e-3)`), defined once in the parameter file; no variable step in identification (intake manifold exercise used 0.01 s, ISCS uses 1 ms).
- Use `Subsystem` blocks (never a separate `.slx` inside the model), `From/Goto`, `Mux/Demux`, `Fcn` for one-line formulas and `MATLAB Function` for algebraic blocks with several inputs (declare sizes). Keep identification models as copies where subsystems are deleted.
- No hard-coded values in blocks; define parameters and initial conditions in the parameter file / structs. Integrator ICs from data (`p_init = p(1)` or `mean(p(1:10))`).
- Measurements are "struct with time" and go into `From Workspace` blocks. Add `Saturation` 20-500 rad/s on `omega_e` while fitting, otherwise `fminsearch` crashes the simulation.
- Wrong `linmod` target (not the normalized model), wrong linearization point and not normalizing the states are the three common mistakes of the linearization step.

## Control theory quick reference (manual Appendix A, B)
- Normalization `x = z/z0`, linearization `d(dx)/dt = A dx + B du`, `dy = C dx + D du` around an equilibrium; with `u0 != 0` the normalized deviations are `du = (u-u0)/u0`, `dy = (y-y0)/y0`.
- Loop gain `L = P*C`; sensitivity `S = 1/(1+L)` (low at low frequency, disturbance rejection); complementary sensitivity `T = L/(1+L)` (low-pass for noise); crossover frequency `wc` (|L| = 1), `t90 ~ 1.7/wc`; phase margin (negative -> unstable; overshoot rule of thumb `~ (71 - PM)/117`, PM in degrees);
  minimum return difference `mu = min|1+L(jw)| = 1/max|S|` (distance to -1; LQR alone has `mu >= 1` and PM >= 60 deg, the observer reduces it); integrating loop -> zero steady-state error; controllable/observable: `rank [B AB ... A^(n-1)B] = n`, `rank [C; CA; ...] = n`.
- LQR: `J = int(x'Qx + u'Ru) dt`, `K = R^-1 B' P` (`lqr(A,B,Q,R)`), `A-BK` is Hurwitz; `R = diag(r1,r2)`: small r = cheap/strong control; `Q = C'C` penalizes the output. Observer by the dual Riccati equation `L = lqr(A',C',B*B',q)'`; needs (A,B) controllable and (A,C) observable. Observer design options: poles 3x faster than `A-BK`, or check the disturbance step response.
- Linear least squares `M x = y + r`, `x = (M'M)^-1 M' y` (`M\y`); condition number `kappa(A) = ||A||*||A^-1||` amplifies errors in `b` by up to `kappa`; `kappa(M'M) = kappa(M)^2`, so normalize data first. Useful commands: `ss, tf, margin, bode, nyquist, lqr, c2d, ssdata, linmod, pade, fminsearch, fmincon`.
