# IdleController

**Idle speed control (ISCS) of a 2.5 l, 5-cylinder SI engine** · team project of *Engine Systems* (ETH Zurich, 151-0567-00, Fall 2026)

| | |
|---|---|
| **Course** | Engine Systems, Prof. C. Onder · exercise supervisors G. Fieni, J. Wüthrich · IDSC |
| **Task** | Model the engine, identify its parameters on test-bench data, linearize it, design an LQGI idle speed controller, compete on cost *J* |
| **Tools** | MATLAB / Simulink R2026b (Control System Toolbox for `lqr`, `c2d`, `margin`) |
| **Team** | 3 to 5 people; the exercise gives up to 0.25 grade points, the grade itself is a 30 min oral exam that also covers the exercise |
| **Manual** | `2026_ExerciseInstructionsManual.pdf` (not in this repo). Page numbers below are the printed ones |
| **State** | **2026-11-23** — see [Status](#1-status) |

> [!IMPORTANT]
> M1 and M2 are done (model error still large). M3: linearization done; two LQGI controllers by William (`M3 Controller/William/`) pass closed-loop tests on the nonlinear plant
> for references of **90-200 rad/s** with load steps and recorded noise. Since 2026-11-16 two more team members contributed: Jeremie added a third LQGI controller (`Jeremie/`, designed at 90 rad/s, ignition-heavy, with its own robustness test `robust_test.m`), and Leon is building a live test of the observer estimation error (`Observer_Test/`). **A load step at references below about 85 rad/s still stalls the model**;
> at 70 rad/s no feedback controller can prevent it (see [Results](#7-results-so-far)). Nothing has been tested on the real engine yet.

## Contents
1. [Status](#1-status) · 2. [Repository layout](#2-repository-layout) · 3. [Quick start](#3-quick-start) · 4. [Data](#4-data) · 5. [The plant model](#5-the-plant-model) · 6. [Procedure per milestone](#6-procedure-per-milestone) · 7. [Results so far](#7-results-so-far) · 8. [Open items](#8-open-items) · 9. [Reference](#9-reference)

---

## 1. Status

| # | Milestone | Deadline | Status | Where |
|:-:|-----------|----------|:------:|-------|
| 1 | Nonlinear plant model in Simulink | Oct 25 | ✅ done | `M1 Model/models/Engine_Plant.slx` |
| 2 | Parameter identification and validation | Nov 8 | ⚠️ done, error ≈ 10 rad/s | `M2 Identification/` |
| 3 | Normalization, linearization, controller working in simulation | Nov 29 | ⚠️ works for 90-200 rad/s, not below | `M3 Controller/`, `M3 Controller/William/` |
| 4 | Controller on the test bench, competition | Dec 10 ¹ | ⬜ not started | candidates: `William/controller1_William.mat`, `controller2_William.mat` |

¹ Class 4 said Dec 13, classes 5 and 6 say Dec 10; the class fact sheet lists the competition on Dec 11.

**Test bench** (ML E44, dSpace ECU): model-validation session week of Nov 2 (done, our recordings are `studentId_*`), controller sessions weeks of Nov 30 and Dec 7. Two controllers per team, the lowest cost *J* wins (pizza voucher).
**Our slot: Mon 2026-11-30, 14:00. Up to 5 controllers can be tested in the session** (so test candidates there and pick the two to submit afterwards). The speed profile is given and **differs in every session**, so do not tune to one profile. The TAs also offered to run a controller file in their simulation beforehand (as suggested in the exercise): worth sending one early for feedback.
Requirement from the TA (2026-11-23, relayed by a teammate): do a **frequency analysis of every controller**; the controller output should be at least two orders of magnitude (the wording is unclear whether higher or lower, and relative to what: **clarify with the TA**; inferred: it concerns noise amplification at high frequency, which is also what drives the throttle activity in *J*).

**Milestone 3 in detail:** linearization at 140 rad/s and a sweep over 70-200 rad/s are done. `M3 Controller/William/` holds the LQGI design script, a closed-loop bench and two controller files.
Other team members' work (since 2026-11-16):
- **Jeremie** (`M3 Controller/Jeremie/`): `LQGI_run_Jeremie.m` designs at 90 rad/s with `Δu_ζ,nom = −30 °CA` (ignition does the fast work, throttle stays calm, which targets the 85-90 % of *J* caused by throttle activity), `qobs = 2e-3`, `r₁ = 0.045`, `qw = 52`; it linearizes itself (cached in `LinModels_Jeremie.mat`, 2-4 min) and saves `controller1_Jeremie.mat`. `robust_test.m` (now `M3 Controller/_Testing/robust_test.m`, shared) is a stricter bench than `test_controller`: 10 random profiles, noise ×1.5 with a different offset per profile, throttle actuator lag, noise ×2, 9 parameter variants, overshoot and steady-state error. Uses a private copy of the bench (`M4 Competition/models/Bench_ISCS_Robust.slx`, created on first use).
- **Leon** (`M3 Controller/Leon/`): `LQR_Controller_Leon.m` (LQR/observer design, moved here from `00. Main/`) and the **observer error graph**, now merged into the shared live test (`_Testing/live_test.m`, see Quick start). Leon's original package is kept in `M3 Controller/_Archive/Observer_Test_Leon_copy/`. The commit log (2026-11-22) says the earlier factor-1000 error in the observer values is fixed; in the merged version the error reads about 0 rad/s at steady state (checked with the template controller, 2026-11-23).
- No results of Jeremie's or Leon's controllers are in the README yet; compare them with `test_controller({...})` or `robust_test({...})`.

---

## 2. Repository layout

```text
IdleController/
├── ISCSData/                  five recordings (see Data)
├── 00. Main/                  working folders, one per milestone
│   ├── M1 Model/              main.m · ISC_Parameters.m
│   │   └── models/            Engine_Plant.slx
│   ├── M2 Identification/     ParID.m · ParID_bench.m · Validation.m · ident_fit.mat · ident_fit_bench.mat
│   │   └── functions/         costVm.m · costTorque.m
│   ├── M3 Controller/         LinNom_bench.m · LinSweep_bench.m
│   │   │                      LinModel_bench.mat · LinModels_bench.mat
│   │   ├── _Models/           Engine_Plant_Norm.slx · Engine_Plant_SteadyState.slx
│   │   ├── _Testing/          ALL scripts that test a controller (shared, see Quick start)
│   │   │                      run_test.m   EDIT AND RUN: set the controller file(s), reference profile and load at the top, press Run
│   │   │                      run_live.m   EDIT AND RUN: live window with graphs, reference slider, load button (+ random load), tick boxes
│   │   │                      test_controller.m   test_controller('controller1_<Name>.mat') -> cost J, minimum speed, plot;
│   │   │                                          options scenario="comp"|"step"|"custom", seed, w_ref, noise, plot; a cell array of files gives a comparison table
│   │   │                      robust_test.m       stress test: 10 profiles, more noise, throttle lag, 9 parameter variants (2-4 min per controller)
│   │   │                      live_test.m         the live window behind run_live.m
│   │   ├── William/           personal folder
│   │   │                      LQGI_run_William.m       design, robustness check, saves the controller file
│   │   │                      controller1_William.mat · controller2_William.mat    competition-format files
│   │   ├── Jeremie/           LQGI_run_Jeremie.m (design at 90 rad/s, −30 °CA)
│   │   │                      controller1_Jeremie.mat · LinModels_Jeremie.mat (cache)
│   │   ├── Leon/              LQR_Controller_Leon.m · Observer_Test_Leon.slx (model used by the LQR script)
│   │   ├── _Archive/          unused old files (see its README.txt): LinNom.m · Linearization_ISC.m · Bench_run_William.m · Bench_William.slx · Controller_William.slx
│   │   └── _Template/         LQGI_run_Template.m · controller1__Template.mat: copy this folder, rename it to your name, rename the script
│   ├── M4 Competition/        models of the test bench and, later, the competition files
│   │   └── models/            Bench_ISCS.slx (bench of test_controller: controller + limits + Engine_Plant_CL) · Engine_Plant_CL.slx (Engine_Plant + omega_e outport)
│   │                          Bench_Live.slx · Engine_Plant_Live.slx (live window: plant with load inputs and extra outputs) · Observer_Test_Live.slx (Bench_Live plus a parallel copy of the observer, used when a controller has observer data)
└── 01. Throttle Area … 07. Load Torque/    first per-block models, kept for reference
      ThrottleArea · ThrottleAMF · IntakeManifold · AMF_model · Engine_Torque (+ ParID, Modelerror) · EngineInertia · LoadTorque
```

Conventions: in every milestone folder the **scripts you run** (`main`, `ParID`, `run_test`, ...) stay at the top, **functions** go into `functions/` and **Simulink models** into `models/` (in M3 `_Models/`; all controller tests are in `M3 Controller/_Testing/`); the scripts add these folders to the path themselves. Shared folders M1-M3 have one owner; personal work goes into a personal subfolder; one person edits a given `.slx` at a time (Simulink files do not merge). `.slxc` and `slprj/` are git-ignored.

---

## 3. Quick start

1. Make the repository the current folder in MATLAB. The main scripts add the four milestone folders to the path themselves:
   `ParID*.m`, `Validation.m`, `LinNom_bench.m`, `LinSweep_bench.m`, `LQGI_run_<Name>.m`, `run_test.m`, `run_live.m`, `main.m` (and the functions `test_controller`, `live_test`).
   `M3 Controller/Leon/LQR_Controller_Leon.m` has no path line, so run `main` first.
2. `main` builds the workspace and **starts with `clear`**. It creates the structs `constants`, `ident` (initial guesses), `init`, `limits`, `simcfg`, `data`.
   `data_name` in `main.m` selects the recording (default `studentId_0003`). `data.meas.T_a.signals.values` etc. are read by the From-Workspace blocks of the plant.
3. Run order:

| Step | Script | Writes | Duration |
|------|--------|--------|----------|
| Identification | `M2 Identification/ParID_bench.m` | `ident_fit_bench.mat` | about 20 min |
| Validation | `M2 Identification/Validation.m` | nothing (table and plots) | about 1 min |
| Linearization at one point | `M3 Controller/LinNom_bench.m` | `LinModel_bench.mat` | short |
| Linearization sweep | `M3 Controller/LinSweep_bench.m` | `LinModels_bench.mat` | short |
| Controller design | `M3 Controller/William/LQGI_run_William.m` (set `ctrl = 1` or `2`) | `William/controller1_William.mat` or `controller2_William.mat` | seconds |
| Closed-loop test of **any** controller file | `M3 Controller/_Testing/run_test.m` (edit the settings at the top, press Run) or `test_controller('controller1_Leon.mat')` | nothing (cost *J*, minimum speed, plot) | about 3 s |

**Live test with sliders** (`M3 Controller/_Testing/run_live.m`): set `ctrlFile` at the top and press Run. If the controller comes with observer data (`<name>_observer.mat`, written by `_Template/LQGI_run_Template.m`), the window also shows the **observer error** `ω_e,obs − ω_e,meas`; without it the plain bench is used. One window opens, runs the nonlinear plant in real time and has
a **reference-speed slider** (the controller sets throttle and ignition by itself), a **Load button** (1.2 kW on/off) and **Random load**, plus tick boxes for what the graphs show
(speed, reference, speed error, throttle, ignition, manifold pressure, engine torque, load torque, air mass flows, load power). It also uses the recorded measurement noise. Close the window to stop.

**Your own controller** (every team member):
1. Copy `M3 Controller/_Template/` to `M3 Controller/<YourName>/` (or use your existing folder) and rename the script to `LQGI_run_<YourName>.m`.
2. Change the tuning parameters at the top, or replace the design section with your own method. The result must be the six competition matrices (`ISCS_Ad/Bd/Cd/Dd`, `ISCS_Tu`, `ISCS_Ty`).
3. Run the script: it saves `controller1_<YourName>.mat` in your folder and calls `test_controller` on it.
4. Compare variants on the same scenario: `test_controller({fileA, fileB, fileC})` prints one line per file (*J*, tracking, throttle activity, minimum speed, RMS error, stall).
   Other tests: `test_controller(file, scenario="step", w_ref=140)`, `seed=2`, `noise=false`, `w_low=100`. Same scenario and seed = comparable *J*.

`test_controller` writes no files and does not clear your workspace. Convention it assumes (inferred from manual Fig. 2.15): controller input = `(ω_ref − ω_e)/ISCS_Ty`, plant input = `ISCS_Tu·(1 + controller output)`.

`ParID.m` and `Validation.m` are the earlier version (fit on `studentId_0002`, result `ident_fit.mat`); the `*_bench` versions feed the linearization.

---

## 4. Data

`ISCSData/`, sampled at 1 kHz. Each file holds a struct `meas`; every signal is `meas.<name>.time` and `meas.<name>.signals.values` ("struct with time", used by the From-Workspace blocks).

| File | Length | Content | Origin |
|------|-------:|---------|--------|
| `quasistatic_0001` | 200 s | slow throttle ramp, no load: static identification | course |
| `dynamic_0002` | 80 s | throttle steps, load 26 % of the time | course |
| `dynamic_0003` | 70 s | throttle steps, load 29 % of the time | course |
| `studentId_0002` | 120 s | throttle staircase, no load, 72-258 rad/s | **our bench, 2026-11-02** |
| `studentId_0003` | 120 s | dynamic, load steps ≈ 1.2 kW, 40 % on, 67-260 rad/s | **our bench, 2026-11-02** |

| Signal | Meaning | Unit |
|--------|---------|------|
| `omega_e` · `omega_e_desired` | engine speed · bench reference speed | rad/s |
| `p_m` · `p_a` · `p_e` | intake manifold · ambient · exhaust pressure | Pa |
| `T_m` · `T_a` | intake manifold · ambient temperature | K |
| `m_dot_alpha` | air mass flow through the throttle | kg/s |
| `u_alpha` · `du_ign` | throttle command · ignition retardation from MBT | % · °CA |
| `u_l` · `P_l` | load on/off · modeled load power | 0/1 · W |
| `U_Bat`, `I_Gen`, `lambda`, `cont_out` | not needed for identification | |

> [!WARNING]
> `du_ign` is exactly 0 in **all five files**. No data validates the ignition-to-torque path (`k_ζ`), so the second controller input rests on the manual's formula only.

---

## 5. The plant model

Causality chain (manual Fig. 2.5, p. 17):

```text
u_alpha → Throttle Area → Throttle Air Mass Flow → Intake Manifold → Engine Air Mass Flow ┐
                                  ↑ p_m, θ_a, p_a            ↑ θ_m       p_m, p_e, ω_e ↓
                 Δu_ζ → Engine Torque Generation → Engine Inertia → ω_e      (Load Torque: P_l, u_l)
```

| Quantity | Description |
|----------|-------------|
| Inputs | `u_α` throttle 0…100 %; `Δu_ζ` ignition retardation from MBT in °CA, **≤ 0** (positive values are replaced by 0 on the bench) |
| States / output | `ω_e`, `p_m` (plus two delays) / `ω_e` |
| Disturbance | `u_l`, `P_l` (≈ 1 kW water boiler, ≈ 1.4 kW mechanical); simulation only, never in the controller or the linear model |
| Range / solver | 70-200 rad/s; fixed-step `ode1`, 1 ms; exactly **two** variable transport delays |

**Equations** (manual p. 18-23)

$$A_\alpha=\alpha_0+\alpha_1u_\alpha,\qquad
\dot m_\alpha=\begin{cases}A_\alpha\dfrac{p_a}{\sqrt{R\vartheta_a}}\dfrac{1}{\sqrt2}, & p_m/p_a<\tfrac12\\[2mm]
A_\alpha\dfrac{p_a}{\sqrt{R\vartheta_a}}\sqrt{2\dfrac{p_m}{p_a}\Big(1-\dfrac{p_m}{p_a}\Big)}, & p_m/p_a\ge\tfrac12\end{cases}$$

$$\dot p_m=\frac{R\vartheta_m}{V_m}\big(\dot m_\alpha-\dot m_\beta\big)\quad\text{(isothermal, no }\kappa\text{)}$$

$$\dot m_e=\frac12\frac{p_m}{R\vartheta_m}\lambda_l V_d\frac{\omega_e}{2\pi},\quad
\dot m_\beta=\frac{\dot m_e}{1+\frac{1}{\lambda\sigma_0}},\quad
\lambda_l=\lambda_{lp}\lambda_{l\omega},\quad
\lambda_{lp}=\frac{V_c+V_d}{V_d}-\frac{V_c}{V_d}\Big(\frac{p_e}{p_m}\Big)^{1/\kappa},\quad
\lambda_{l\omega}=\gamma_0+\gamma_1\omega_e$$

$$T_e=\big(e\,p_{m\varphi}-p_{me0f}-p_{me0g}\big)\frac{V_d}{4\pi},\qquad
e\,p_{m\varphi}=(\eta_0+\eta_1\omega_e)\,e_\zeta\,\frac{H_l}{V_d}\,\frac{4\pi}{\lambda\sigma_0}\,\frac{\dot m_\beta(t-\tau_{IPS})}{\omega_e(t-\tau_{IPS})}$$

$$e_\zeta=1-k_\zeta\,\Delta u_\zeta(t-\tau_{seg}/2)^2,\quad
\tau_{IPS}=\frac{2\pi}{\omega_e},\quad \tau_{seg}=\frac{4\pi}{5\omega_e},\quad
p_{me0f}=\frac{4\pi\beta_0}{V_d},\quad p_{me0g}=p_e-p_m$$

$$\dot\omega_e=\frac{T_e-T_l}{\Theta_e},\qquad T_l=\frac{P_l}{\omega_e\,\eta_{gen}}$$

**Parameters**

| Parameter | Unit | Value / initial guess | Identified (bench fit) |
|-----------|------|----------------------:|-----------------------:|
| `α₀` | m² | 3e-6 | 1.776e-6 |
| `α₁` | m²/% | 6e-6 | 5.801e-6 |
| `V_m` | m³ | 7e-3 | 7.5e-3 (hardly moved) |
| `γ₀` | – | 0.6 | 0.5371 |
| `γ₁` | s/rad | 0.002 | 0.0018 |
| `η₀` | – | 0.3 | 0.4282 |
| `η₁` | s/rad | −3e-4 | −5.096e-4 |
| `β₀` | Nm | 7 | 11.13 |
| `Θ_e` | kg m² | 0.2 | 0.1848 |

Known constants: `R = 287` J/(kg K), `V_d = 2.48e-3` m³, `V_c = 2.48e-4` m³, `κ = 1.35`, `σ₀ = 14.7` (table; the text says 14.67), `H_l = 42.5e6` J/kg, `k_ζ = 2.3345e-4` 1/(°CA)², `η_gen = 0.7`, `λ = 1`.

**Simulink models in the repo**

| Model | Role |
|-------|------|
| `Engine_Plant.slx` | physical units; replays `T_a, p_a, T_m, p_e, P_l, u_l` from `data.meas`; with `simcfg.omega_meas = 0` it simulates its own speed. MATLAB Function blocks read workspace constants only if declared as **Parameter data** |
| `Engine_Plant_CL.slx` | copy of `Engine_Plant` with an extra outport `omega_e`, to close the loop |
| `Engine_Plant_Norm.slx` | normalized (Fig. 2.11), delays as 4th-order Padé State-Space blocks (frozen at the linearization speed), 10 states, no load. Needs `ident, omega_e_nom, u_alpha_nom, delta_u_nom, Q_nom, p_m_nom, theta_a_nom, p_a_nom, theta_m_nom, p_e_nom, A1..D1, x0_1, A2..D2, x0_2` (set by `LinNom_bench.m`) |
| `Engine_Plant_SteadyState.slx` | finds the steady state for a given `u_alpha_nom` (used by `fzero`) |

---

## 6. Procedure per milestone

<details open>
<summary><b>Milestone 2: identification (manual p. 25-30)</b></summary>

Modular, linear least squares whenever possible:

1. **`α₀, α₁`** (linear LS, quasistatic data): `A_α = ṁ_α·√(2Rϑ_a)/p_a` against `u_α`; requires `p_m/p_a ≤ 0.5`.
2. **`γ₀, γ₁`** (linear LS, quasistatic data): `λ_lω = (4πR/V_d)·ϑ_m ṁ_α/(p_m ω_e λ_lp)·(1+1/(λσ₀))`.
3. **`V_m`**: `fminsearch` on `Σ(p_m − p̂_m)²`, throttle flow from the model, measured `ω_e` fed in.
4. **`η₀, η₁, β₀, Θ_e`**: nonlinear fit (`fminsearch` / `fmincon` with bounds, e.g. `Θ_e > 0`), `ω_e` saturated to 20-500 rad/s. `η₀` partly compensates for `β₀`, so values need not be physical.
5. **Validate** on data not used for the fit.

Class hints: use the simulated `p_m` (feedback) in the identification, not the measured one, otherwise the optimizer diverges · initialize integrators from the data (`p(1)` or `mean(p(1:10))`) · fixed step sizes only ·
with the course data, fit on `dynamic_0003` and validate on `dynamic_0002` (the other way round failed in earlier years) · normalize data before least squares (`cond(MᵀM) = cond(M)²`) · sensible start values (`V_m` ≈ 3 × `V_d`).
</details>

<details open>
<summary><b>Milestone 3: normalization and linearization (p. 31-32)</b></summary>

1. Pick nominal inputs and run to steady state (states, outputs, delay times).
2. Keep only two delay blocks; replace each by a **4th-order Padé** (`pade`, State-Space block, initial state `x₀ = −A\B`); normalize input and output of each Padé element but **not** its states.
3. Build the normalized model: every input, state and output divided by its nominal value (Fig. 2.11).
4. `linmod(model, x0, [1;1])` on the **normalized** model. Expect 10 states, 2 inputs, 1 output, D = 0; check the step response and `cond`.

Class hints: normalize first, linearize afterwards · the linearization point must be a true equilibrium · all matrix entries should be around 1 · to compare with the nonlinear model, denormalize and add the nominal point back (Fig. 1.3).
</details>

<details open>
<summary><b>Controller specifications (p. 32-33)</b></summary>

| Requirement | Value |
|-------------|-------|
| Reference speed | 70-200 rad/s |
| Disturbance | load switched randomly |
| Steady-state error (constant load, reference step) | zero |
| Minimum speed | ≥ 50 rad/s (stall) |
| Overshoot after a load step | reasonable |
| Minimum return difference | μ = min \|1 + L(jω)\| ≥ 0.5 |
| Crossover of L = P·C | preferably ≤ 4 rad/s (noisy signals) |
| Sampling | T_s = 1 ms, with T_s < π / (5 ω_c) |
</details>

<details open>
<summary><b>LQGI design (p. 33-36)</b></summary>

- **Extension** (integrator on the throttle channel): `Ae = 0`, `Be = [1 0]`, `Ce = [K₁; 0]`, `De = I`.
- **Extended plant:** `A = [As, Bs·Ce; 0, Ae]`, `B = [Bs·De; Be]`, `C = [Cs, Ds·Ce]`.
- **Gains:** `K = lqr(A, B, CᵀC, diag(r₁, r₂))`, `L = lqr(Aᵀ, Cᵀ, B·Bᵀ, q)ᵀ` (small `q` = fast observer, trust the measurement).
- **Controller** (`Dc = 0`, input `e = ω_ref − ω`):

  `ISCS_Ac = [Ae, −Be·K; 0, A−B·K−L·C]` · `ISCS_Bc = [Be·Dc; −L]` · `ISCS_Cc = [Ce, −De·K]` · `ISCS_Dc = De·Dc`

- **Discretize** with Tustin (`c2d`) at 1 ms. Guideline: observer poles about 3× faster than those of `A−BK`. Test with band-limited white noise.
- The ignition should return to its nominal value after the throttle has adapted (Fig. 2.3).
</details>

<details open>
<summary><b>Milestone 4: competition (p. 39)</b></summary>

Files `controller1.mat` (and `controller2.mat`) containing:

| Variable | Size / value |
|----------|--------------|
| `ISCS_Ad`, `ISCS_Bd`, `ISCS_Cd`, `ISCS_Dd` | 12×12, 12×1, 2×12, 2×1 |
| `ISCS_Tu` | `diag(u_α,nom, Δu_ζ,nom)` |
| `ISCS_Ty` | `ω_e,nom` |

$$J=\int\Big(0.2\,k_{max}^{2.5}\,\lvert\omega_e-\omega_{e,ref}\rvert+\Big\lvert\frac{du_\alpha}{dt}\Big\rvert\Big)dt,\qquad k_{max}=\frac{1}{1-k_\zeta\,\Delta u_{\zeta,nom}^2}$$

`u_α` in %, limited to 0…100. A large `|Δu_ζ,nom|` costs fuel (`k_max`) but gives the ignition more authority: a design trade-off.
</details>

---

## 7. Results so far

### Identification and model quality
Simulated against measured speed and manifold pressure (RMSE):

| Fit / data | ω_e | p_m |
|------------|----:|----:|
| `ident_fit.mat` on `studentId_0002` (fit data) | 3.51 rad/s | 1262 Pa |
| `ident_fit.mat` on `studentId_0003` | 8.10 rad/s | 1640 Pa |
| bench fit, `studentId_0003` first 60 s (fit) | 9.7 rad/s | 987 Pa |
| bench fit, `studentId_0003` last 60 s (validation) | 9.6 rad/s | 2063 Pa |

Shape and timing of `ω_e` and `p_m` agree. The error is a **speed-dependent offset** (simulated minus measured speed):

| Measured speed | 77 | 133 | 171 | 201 | 242 rad/s |
|----------------|---:|----:|----:|----:|----------:|
| Offset | +4.0 | +3.3 | −2.6 | −7.5 | −15 rad/s |

Above 200 rad/s the γ fit is an extrapolation. At 1 % throttle the simulated `p_m` is about 2200 Pa too low. The model oscillates at low speed where the data does not. The load is not the main error source.

### Linearization (`LinModel_bench.mat`, `LinModels_bench.mat`)
- Nominal point: ω_e = 140 rad/s, Δu_ζ,nom = −7 °CA, u_α,nom = 2.367 %, p_m = 19785 Pa. System 10×10, **stable** (largest real part −1.24, slowest pair −1.24 ± 1.48j); it behaves like a second-order system (Hankel values), the Padé states are fast.
- `cond(A) = 1.55e19` (Padé companion form), 1.9e4 after `balance`. **Always balance before designing.**
- Normalized DC gain: **0.73** from `u_α`, **−0.017** from `Δu_ζ`. At −7 °CA the ignition has almost no authority (more at −15 °CA, at higher `k_max` cost).

| Speed [rad/s] | 70 | 100 | 140 | 200 |
|---------------|---:|----:|----:|----:|
| Largest real part of A (Δu_ζ,nom = −7) | −0.36 | −0.72 | −1.24 | −2.14 |
| u_α,nom [%] (Δu_ζ,nom = −7) | 0.904 | 1.493 | 2.367 | 3.887 |

Stable at all 8 sweep points; the 70 rad/s points had not settled after 30 s (drift ≈ 1.2 rad/s).

### Why the first controller stalled (2026-11-16)
The first LQGI design (linear model at 140 rad/s without load, `r₁ = 10`, `r₂ = 10`, `q_w = 100`, `q_obs = 1e-6`; min |1+L| = 0.864, crossover 3.03 rad/s) stalled for references ≤ 90 rad/s with load. Cause:

- The load is a **constant-power load**, `T_l = P_l/(ω_e η_gen)`: its torque rises when the speed falls. Linearized, it adds `+P_l/(ω_e² η_gen Θ_e)` to the speed equation, which the load-free linear model does not contain.

| Speed [rad/s] | 70 | 100 | 140 | 200 |
|---------------|---:|----:|----:|----:|
| Slowest plant pole, load off [1/s] | −0.36 | −0.72 | −1.24 | −2.14 |
| Slowest plant pole, 1.2 kW load on [1/s] | **+0.55** | −0.27 | −1.01 | −1.33 |
| First controller, load on: closed loop / min \|1+L\| | **unstable** / 0.16 | stable / 0.52 | stable / 0.85 | stable / 0.90 |

- **Physical limit of the model:** a 1.2 kW step at 70 rad/s is ≈ 24 Nm on 0.185 kg m², about −130 rad/s². Even an ideal reaction (throttle to 100 % and ignition to MBT at the instant of load-on) only keeps 61.8 rad/s; with a 50 ms reaction delay 50.8 rad/s; a 20 % throttle step with 50 ms delay stalls. At 100 rad/s a 5 % throttle step is enough (minimum 76 rad/s). **No feedback controller can survive a 1.2 kW load step at 70 rad/s in this model.**
- The recordings contain no load-on event below 129 rad/s (at fixed throttle the real engine dipped by 60-85 rad/s from 130-150 rad/s and kept running), so the loaded low-speed range is **not validated by data**.

### LQGI design (`William/LQGI_run_William.m`)
The design model is now a low-speed operating point of `LinModels_bench.mat` **with the linearized load switched on**, and every design is checked on all 8 cases (70/100/140/200 rad/s, load off and on).
Several hundred designs (design point, load, weights, observer speed) were scanned on these linear criteria; 238 that met them were tested with load steps on the nonlinear plant, a shortlist with recorded noise; two were kept:

| | Controller 1 (`ctrl = 1`) | Controller 2 (`ctrl = 2`) |
|---|---|---|
| Design point | 100 rad/s, `Δu_ζ,nom = −15`, 1.8 kW load | 70 rad/s, `Δu_ζ,nom = −15`, 1.8 kW load |
| Weights `r₁ / r₂ / q_w / q_obs` | 3 / 1 / 100 / 1e-6 | 10 / 10 / 100 / 1e-6 |
| `ISCS_Tu`, `ISCS_Ty` | diag(1.562 %, −15 °CA), 100 rad/s | diag(0.951 %, −15 °CA), 70 rad/s |
| Stable in all 8 cases | yes | yes |
| Worst min \|1+L\| (need ≥ 0.5) | 0.504 (70 rad/s, load on) | 0.703 |
| Crossover | 4.8-5.8 rad/s | 5.0-6.1 rad/s |
| Character | lowest cost in the tests, uses the ignition strongly | larger margin, uses the ignition less |

Notes: the crossover is **above the 4 rad/s hint** of the manual (p. 33); the hint is about noise, and the designs were tested with recorded noise instead. `Δu_ζ,nom = −15` costs `k_max^2.5 = 1.144` in *J* (1.029 at −7) but gives the ignition authority.
The "observer 3× faster" guideline is not reachable (limited by the plant's slow pole). *Inferred:* the controller works on normalized deviations from the nominal point (Fig. 2.15, p. 39); the manual does not state this.

### Closed loop on the nonlinear plant (`M3 Controller/_Testing/test_controller.m`, `models/Bench_ISCS.slx`)
The table below was produced with the first personal bench; `test_controller` reproduces it (same J = 680 for controller 1, seed 1).
Bench: discrete controller from the `.mat` file → one sample delay → `ISCS_Tu` → limits (throttle 0-100 %, ignition −30…0 °CA) → `Engine_Plant_CL.slx`; constant ambient conditions; 1.2 kW load switched by the script;
measurement noise = the speed recorded on the bench (`studentId_0002`) minus its 0.2 s moving average (std 0.21 rad/s, 7 % of its power above 50 Hz). Cost *J* evaluated from 15 s to 119 s.

| Test (104 s evaluated) | Controller 1 | Controller 2 |
|------|-------------:|-------------:|
| Random references 90-200 rad/s + random load, seed 1 / 2 / 3: *J* | 680 / 777 / 786 | 796 / 913 / 915 |
| … minimum speed [rad/s] | 60.7 / 68.1 / 59.3 | 61.3 / 67.3 / 62.5 |
| Constant 90 rad/s, five load steps: *J*, speed range | 583, 60.8-104.3 | 660, 60.4-106.1 |
| Constant 140 rad/s, five load steps: *J*, speed range | 554, 127.7-149.2 | 642, 126.1-152.8 |
| Constant 200 rad/s, five load steps: *J*, speed range | 552, 191.6-207.5 | 640, 187.2-211.8 |
| Stall (< 50 rad/s) | none | none |

- For comparison, the first controller in the same kind of random test: *J* = 753 / 728 / 1207 with the speed below 50 rad/s in all three runs, and a stall at a load step at 90 rad/s.
- **About 85-90 % of *J* is throttle activity** (e.g. 619 of 680), mostly caused by measurement noise. A faster observer survives lower speeds but raises this term; a slower one lowers it but stalls at load steps below 100 rad/s. The recorded noise is used instead of white noise because it is mostly low-frequency.
- **Limits that remain:** a load step at a reference of 80 rad/s stalls both controllers (scan without noise); at 70 rad/s it cannot be survived at all. The ignition lower limit of −30 °CA is an **assumption** (the manual gives none); both controllers were also run with −20 °CA without a stall.
  The ignition-to-torque path is not validated by data (`du_ign = 0` in all recordings), and both controllers rely on it.

### Preliminary exercise (intake manifold, not in this repo)
Done: `V_m = 0.00923 m³`, `λ_lω = 1.0379`, eigenvalues of A −2.474 and −4.093 1/s, validation RMSE ≈ 850 Pa. Its scripts (`Main`, `Parameters`, `ParID`, `Modelerror`, `Linearization`, `ParIdDemo/` with `fminsearch`) are the template for M2 and M3.

---

## 8. Open items

- [x] Shared closed-loop bench with load, recorded noise and cost *J*: `M3 Controller/_Testing/test_controller.m` + `models/Bench_ISCS.slx` (any controller file, several files in one table).
- [ ] Each team member designs a controller in their own folder and compares it with `test_controller({...})`. Done so far: William (2 controllers), Jeremie (1); Leon is working on the observer test; a fourth member folder is still to be created from `_Template/`.
- [ ] Run `robust_test({'controller1_William.mat','controller2_William.mat','controller1_Jeremie.mat'})` and put the comparison into section 7.
- [ ] Observer-error graph: only `_Template/LQGI_run_Template.m` saves the observer matrices (`<name>_observer.mat`). Add the same two lines to `LQGI_run_William.m` and `LQGI_run_Jeremie.m` to get the graph for their controllers.
- [x] Controller that works for references of 90-200 rad/s with load steps (`controller1_William.mat`, `controller2_William.mat`).
- [ ] Load steps at references below about 85 rad/s still stall the model. Ask the supervisors whether the competition switches the load on at 70 rad/s, and whether the real engine behaves like the model there (not covered by any recording).
- [ ] Ask the supervisors for the real lower limit of the ignition retardation (−30 °CA is assumed) and how the test bench applies `ISCS_Tu` / `ISCS_Ty` (assumed: normalized deviations from the nominal point).
- [ ] Frequency analysis of every candidate (Bode of controller, loop `L`, sensitivity; high-frequency gain from speed noise to throttle) and clarify the TA's "two orders of magnitude" criterion.
- [ ] Optional: send `controller1_Jeremie.mat` to the TA for a check in their simulation before Nov 30.
- [ ] Add the `robust_test` comparison (2026-11-23, Jérémie: mean *J* over 10 profiles / stalls out of 11 stress cases: Jeremie 534 ± 52 / 0, William c1 1019 ± 71 / 2, William c2 1160 ± 76 / 2; stalls with throttle lag and V_m ×1.5). Not comparable to the *J* values in section 7 (noise ×1.5, other profiles).
- [x] Duplicate `test_controller.m`/`live_test.m` copies removed; Leon's observer plot merged into `_Testing/live_test.m`, old package in `_Archive/` (2026-11-23).
- [ ] Decide as a team which up to five controllers to test in the Nov 30 session, and which two to submit, then save them as `controller1.mat` and `controller2.mat` (the names the competition expects) and test them at the bench sessions (weeks of Nov 30 and Dec 7).
- [ ] Reduce the throttle activity caused by noise (85-90 % of *J*), without losing the low-speed robustness.
- [ ] Reduce the identification error (fit restricted to 70-200 rad/s; `V_m` did not move from its guess).
- [ ] Ask the supervisors for a recording with ignition steps, or accept the manual's formula for `Δu_ζ`.
- [ ] Manual p. 23 bonus question: which part accounts for the auxiliary power (probably `β₀`/`η₀`, unconfirmed). Manual p. 31 bonus question: why are the Padé states not normalized.
- [ ] `resources/project/` shows staged deletions in `git status`; `IdleController.prj` is an empty MATLAB project.

---

## 9. Reference

<details>
<summary><b>Course facts</b> (class fact sheet, exercise classes 1-6)</summary>

- **Textbook:** Guzzella / Onder, *Introduction to Modeling and Control of Internal Combustion Engine Systems*, 2nd ed. 2010, with corrigendum. Documents are on Moodle and on the IDSC course page. Lecture Mon 8:15, exercise class Mon 12:15 (ML F 36). A recap on modeling and control was held on Oct 7.
- **Exercises:** theory sheets 1-6 are solved **individually** (solutions on Moodle). The ISCS exercise is group work (at least 3 people; at least 2 who know MATLAB/Simulink and 2 who know control systems).
- **Exam:** oral, 30 min, English or Swiss German. Scope: theory sheets, engine components and operating principles, modeling equations and their assumptions (reproduce or derive them), main engine control tasks, back-of-envelope numbers (atmospheric pressure, lower heating value, fuel properties). **The ISCS exercise is relevant for the exam.**
- **Competition:** the load disturbance is unknown in advance, so the controller must be robust and not tuned to a known profile.
</details>

<details>
<summary><b>Modeling method and Willans model</b> (class 1 and class 5)</summary>

**Model types:** black box (experiments only), white box (physics, known parameters), gray box (physics plus identified parameters; used here).

**Reservoir approach:**
1. Define boundaries, inputs, outputs.
2. Choose reservoirs and level variables (kinetic energy → speed, mass/thermal → pressure, temperature). Fast dynamics → algebraic, relevant → differential equation, slow → constant.
3. Draw the causality diagram (same structure as the Simulink model).
4. Conservation laws: d/dt content = Σ inflows − Σ outflows.
5. Algebraic flow equations as functions of the level variables.
6. Identify parameters (static: least squares, e.g. `λ`; dynamic: `fminsearch`, e.g. `V_m`, `Θ`).
7. Validate with other experiments.

**Willans approximation:** `p_me = e·p_mφ − p_me0`, with `e` the thermodynamic conversion efficiency and `p_me0` friction plus gas-exchange losses; affine in the fuel energy per cycle. Typical values: `e ≈ 0.5` at full load; `p_me0 ≈ 1 bar` for a gasoline engine at idle (closed throttle, pumping losses), `≈ 0.1 bar` at full load. Zero torque while injecting fuel is idling (only losses are covered); efficiency rises with load.
Here: constant friction `β₀`, and `e = η₀ + η₁ω_e` times the ignition efficiency `e_ζ`.
</details>

<details>
<summary><b>Simulink and MATLAB practices</b> (classes 2 and 4)</summary>

- Fixed-step solver with an explicit step, defined once in the parameter file: `simset('SrcWorkspace','current','Solver','ode1','FixedStep',1e-3)`. The intake manifold exercise used 0.01 s, the ISCS uses 1 ms. No variable step in identification.
- Use `Subsystem` blocks (never a separate `.slx` inside the model), `From/Goto`, `Mux/Demux`, `Fcn` for one-line formulas, `MATLAB Function` for algebraic blocks with several inputs (declare the sizes). Keep identification models as copies in which subsystems are deleted.
- No hard-coded values in blocks; define parameters and initial conditions in the parameter file or structs. Integrator initial conditions from data.
- Measurements are "struct with time" and go into `From Workspace` blocks. Add a `Saturation` of 20-500 rad/s on `ω_e` while fitting, otherwise `fminsearch` crashes the simulation.
- The three common linearization mistakes: wrong `linmod` target (not the normalized model), wrong linearization point, states not normalized.
- Do not define the same variable in two parameter files (the last definition silently wins).
</details>

<details>
<summary><b>Control theory quick reference</b> (manual Appendix A and B)</summary>

- **Normalization** `x = z/z₀`; **linearization** `d(δx)/dt = A δx + B δu`, `δy = C δx + D δu` around an equilibrium. With `u₀ ≠ 0`: `δu = (u − u₀)/u₀`, `δy = (y − y₀)/y₀`.
- **Loop gain** `L = P·C`. **Sensitivity** `S = 1/(1+L)` (low at low frequency: disturbance rejection). **Complementary sensitivity** `T = L/(1+L)` (low-pass: noise). **Crossover** `ω_c` (|L| = 1), `t₉₀ ≈ 1.7/ω_c`.
  **Phase margin**: negative means unstable; overshoot rule of thumb `≈ (71° − PM)/117°`. **Minimum return difference** `μ = min|1+L(jω)| = 1/max|S|` (distance to −1; an LQR alone has `μ ≥ 1` and PM ≥ 60°, the observer reduces it).
  An integrating loop gives zero steady-state error. Controllable: `rank[B AB … Aⁿ⁻¹B] = n`; observable: `rank[C; CA; …] = n`.
- **LQR:** `J = ∫(xᵀQx + uᵀRu)dt`, `K = R⁻¹BᵀP` (`lqr(A,B,Q,R)`), `A−BK` is Hurwitz. `R = diag(r₁, r₂)`: small `r` = strong control. `Q = CᵀC` penalizes the output. **Observer** by the dual Riccati equation `L = lqr(Aᵀ,Cᵀ,B·Bᵀ,q)ᵀ`; needs (A,B) controllable and (A,C) observable. Options: observer poles 3× faster than `A−BK`, or check the disturbance step response.
- **Least squares** `M x = y + r`, `x = (MᵀM)⁻¹Mᵀy` (`M\y`). Condition number `κ(A) = ‖A‖·‖A⁻¹‖` amplifies errors in `b` by up to `κ`; `κ(MᵀM) = κ(M)²`, so normalize data first.
- **Useful commands:** `ss, tf, margin, bode, nyquist, lqr, c2d, ssdata, linmod, pade, fminsearch, fmincon`.
</details>
