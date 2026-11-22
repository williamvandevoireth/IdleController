function fig = live_test(ctrlFile, opt)
%LIVE_TEST Live test of Observer_Test_Leon with observer estimation error.
%
%   live_test('controller1_Leon.mat')
%   live_test(file, P_on=1200, rand_on=[4 12], rand_off=[4 12], ...
%             du_min=-30, window=40)
%
%   Preserves the original live controls and plots and additionally plots
%   e_obs = omega_e_obs - omega_e_meas.
%
%   The Outport omega_e_obs carries the 11-element shifted/scaled state vector
%   after Sum1 (+ ones) and Gain1 (* ISCS_Ty), NOT a scalar speed.
%   For the speed estimate, this function reads the raw deviation-state vector
%   directly from the Observer block and calculates
%       omega_e_obs = ISCS_Ty * (1 + Obs_Cy * x_hat_deviation).
%   The Outport omega_e_meas carries the measured speed (source: Plant).

arguments
    ctrlFile
    opt.P_on     (1,1) double = 1200
    opt.rand_on  (1,2) double = [4 12]
    opt.rand_off (1,2) double = [4 12]
    opt.du_min   (1,1) double = -30
    opt.window   (1,1) double = 40
end

%% Project paths

here = fileparts(mfilename('fullpath'));

% live_test.m liegt in Observer_Test/functions
observerRoot = fileparts(here);

% Übergeordnetes Projektverzeichnis finden
projectRoot = findProjectRoot(observerRoot);

% Bisherige Projektordner
folders = fullfile(projectRoot, ...
    {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});

folders = [folders, ...
    fullfile(folders,'functions'), ...
    fullfile(folders,'models')];

% Neue Observer_Test-Ordner ergänzen
folders = [folders, { ...
    observerRoot, ...
    fullfile(observerRoot,'functions'), ...
    fullfile(observerRoot,'models')}];

addpath(strjoin(folders(isfolder(folders)), pathsep), '-end');

%% Controller file

ctrlFile = char(ctrlFile);

if ~isfile(ctrlFile)
    error('Controller file "%s" not found.',ctrlFile);
end

c = load(ctrlFile);
need = {'ISCS_Ad',[12 12]; ...
        'ISCS_Bd',[12 1]; ...
        'ISCS_Cd',[2 12]; ...
        'ISCS_Dd',[2 1]; ...
        'ISCS_Tu',[2 2]; ...
        'ISCS_Ty',[1 1]};
for i = 1:size(need,1)
    if ~isfield(c,need{i,1}) || ~isequal(size(c.(need{i,1})),need{i,2})
        error('%s: variable %s missing or not %s.', ...
            ctrlFile,need{i,1},mat2str(need{i,2}));
    end
end

% The combined ISCS realization has one integrator state plus the observer
% states. In this design, ISCS_Ad is 12x12 and Obs_Cy must be 1x11.
nObserver = size(c.ISCS_Ad,1) - 1;
if ~isfield(c,'Obs_Cy')
    error(['%s does not contain Obs_Cy. Rerun LQGI_run after saving ', ...
        'Obs_Cy in the controller MAT-file.'],ctrlFile);
end
if ~isequal(size(c.Obs_Cy),[1 nObserver])
    error('Obs_Cy must be 1x%d, but its size is %s.', ...
        nObserver,mat2str(size(c.Obs_Cy)));
end

%% Workspace of the models (same setup as test_controller.m)
m = getMain();
S = load('LinModels_bench.mat');
M = S.M;
Lm = load('LinModel_bench.mat');

sameDu = find([M.delta_u_nom] == c.ISCS_Tu(2,2));
if isempty(sameDu)
    sameDu = 1:numel(M);
end
[~,j] = min(abs([M(sameDu).omega_e_nom] - c.ISCS_Ty));
k = sameDu(j);

init = m.init;
init.omega_e0 = c.ISCS_Ty;
init.p_m0 = M(k).p_m_nom;
init.du_init = c.ISCS_Tu(2,2);
init.q_init = M(k).Q_nom;

simcfg = m.simcfg;
simcfg.omega_meas = 0;

data = m.data;
t = data.meas.time(:);
N = numel(t);
c1 = ones(N,1);
data.meas.T_a.signals.values = Lm.theta_a_nom*c1;
data.meas.p_a.signals.values = Lm.p_a_nom*c1;
data.meas.T_m.signals.values = Lm.theta_m_nom*c1;
data.meas.p_e.signals.values = Lm.p_e_nom*c1;

q = load(fullfile(fileparts(which('main')),'..','..', ...
    'ISCSData','studentId_0002.mat')).meas;
wq = q.omega_e.signals.values(:);
noise = wq(1:N) - movmean(wq(1:N),200);

V = struct('data',data, ...
    'init',init, ...
    'ident',S.ident, ...
    'constants',m.constants, ...
    'limits',m.limits, ...
    'simcfg',simcfg, ...
    'ISCS_Ad',c.ISCS_Ad, ...
    'ISCS_Bd',c.ISCS_Bd, ...
    'ISCS_Cd',c.ISCS_Cd, ...
    'ISCS_Dd',c.ISCS_Dd, ...
    'ISCS_Tu',c.ISCS_Tu, ...
    'ISCS_Ty',c.ISCS_Ty, ...
    'Obs_Cy',c.Obs_Cy, ...
    'Ts',1e-3, ...
    'noise_ts',struct('time',[], ...
        'signals',struct('values',noise,'dimensions',1)), ...
    'du_min',opt.du_min, ...
    'P_on',opt.P_on);

f = fieldnames(V);
for i = 1:numel(f)
    assignin('base',f{i},V.(f{i}));
end

%% Model: keep the Simulink model loaded and its window visible

mdl = 'Observer_Test_Live';

modelFile = fullfile(observerRoot,'models',[mdl '.slx']);

if ~bdIsLoaded(mdl)
    load_system(modelFile);
else
    if strcmp(get_param(mdl,'SimulationStatus'),'running')
        set_param(mdl,'SimulationCommand','stop');
    end
end

open_system(mdl);

w0 = min(max(round(c.ISCS_Ty),70),200);
set_param([mdl '/ref_w'],'Value',num2str(w0));
set_param([mdl '/load_on'],'Value','0');

%% Signals: key, label, graph (axis), line colour, shown at start
sig = { ...
    'speed', 'speed \omega_e',                                      1, [0.85 0.10 0.10], true; ...
    'ref',   'reference',                                           1, [0 0 0],          true; ...
    'err',   'speed error (ref - speed)',                           1, [0.93 0.69 0.13], false; ...
    'eobs',  'observer error (\omega_{e,obs} - \omega_{e,meas})',   8, [0 0.45 0.74],    true; ...
    'ua',    'throttle u_\alpha [%]',                               2, [0 0.45 0.74],    true; ...
    'du',    'ignition \Delta u_\zeta',                            3, [0.47 0.67 0.19], true; ...
    'pm',    'manifold pressure p_m',                               4, [0.49 0.18 0.56], false; ...
    'Te',    'engine torque T_e',                                   5, [0.85 0.33 0.10], false; ...
    'Tl',    'load torque T_l',                                     5, [0.30 0.75 0.93], false; ...
    'mda',   'air flow throttle',                                   6, [0 0.45 0.74],    false; ...
    'mdb',   'air flow engine',                                     6, [0.85 0.33 0.10], false; ...
    'Pl',    'load power P_l',                                      7, [0.5 0.5 0.5],    true};

axTitle = {'Speed [rad/s]', ...
    'Throttle [%]', ...
    'Ignition retardation [degCA]', ...
    'Manifold pressure [kPa]', ...
    'Torque [Nm]', ...
    'Air mass flow [g/s]', ...
    'Load power [W]', ...
    'Observer error [rad/s]'};
nS = size(sig,1);
nA = numel(axTitle);

%% Window: controls left, graphs right
st = struct('model',mdl, ...
    'rand_on',opt.rand_on, ...
    'rand_off',opt.rand_off, ...
    'next',inf, ...
    'win',opt.window, ...
    'du_nom',c.ISCS_Tu(2,2), ...
    'Pon',opt.P_on, ...
    'sig',{sig}, ...
    'axTitle',{axTitle}, ...
    'Obs_Cy',c.Obs_Cy, ...
    'ISCS_Ty',c.ISCS_Ty);

fig = uifigure('Name',['live_test: ' nm_of(ctrlFile)], ...
    'Position',[40 40 1200 700]);
G = uigridlayout(fig,[1 2]);
G.ColumnWidth = {310,'1x'};
G.Padding = [8 8 8 8];

L = uigridlayout(G,[7 1]);
L.RowHeight = {22,40,40,26,'1x',54,34};
L.Padding = [2 2 2 2];
L.RowSpacing = 6;

st.refT = uilabel(L, ...
    'Text',sprintf('Reference speed: %d rad/s',w0), ...
    'FontWeight','bold');
st.ref = uislider(L, ...
    'Limits',[70 200], ...
    'Value',w0, ...
    'MajorTicks',70:30:190);
st.load = uibutton(L,'state', ...
    'Text','Load OFF (click to switch on)', ...
    'FontSize',14);
st.rnd = uicheckbox(L, ...
    'Text',sprintf('Random load (on %g-%g s, off %g-%g s)', ...
    opt.rand_on,opt.rand_off));

pn = uipanel(L,'Title','Show in graphs');
pg = uigridlayout(pn,[nS 1]);
pg.RowHeight = repmat({20},1,nS);
pg.Padding = [6 2 2 2];
pg.RowSpacing = 1;
st.cb = gobjects(1,nS);
for i = 1:nS
    st.cb(i) = uicheckbox(pg, ...
        'Text',sig{i,2}, ...
        'Value',sig{i,5});
end

st.info = uilabel(L, ...
    'Text','starting ...', ...
    'WordWrap','on', ...
    'VerticalAlignment','top');
st.stop = uibutton(L,'push', ...
    'Text','Stop and close', ...
    'BackgroundColor',[0.85 0.3 0.3], ...
    'FontColor','w');

st.Rg = uigridlayout(G,[nA 1]);
st.Rg.RowSpacing = 4;
st.Rg.Padding = [2 2 2 2];
st.ax = gobjects(1,nA);
for a = 1:nA
    st.ax(a) = uiaxes(st.Rg);
    grid(st.ax(a),'on');
    title(st.ax(a),axTitle{a},'FontSize',10);
    st.ax(a).FontSize = 9;
end
xlabel(st.ax(nA),'t [s]');
title(st.ax(1), ...
    [axTitle{1} '  -  ' strrep(nm_of(ctrlFile),'_','\_')], ...
    'FontSize',10);

st.ln = gobjects(1,nS);
for i = 1:nS
    st.ln(i) = animatedline(st.ax(sig{i,3}), ...
        'Color',sig{i,4}, ...
        'DisplayName',sig{i,2}, ...
        'LineWidth',1.2, ...
        'MaximumNumPoints',20000);
end
for a = [1 5 6 8]
    legend(st.ax(a),'Location','northeast','FontSize',8,'Box','off');
end

fig.UserData = st;
updateLayout(fig);

st.ref.ValueChangingFcn = @(~,ev) onRef(fig,ev.Value);
st.ref.ValueChangedFcn = @(s,~) onRef(fig,s.Value);
st.load.ValueChangedFcn = @(b,~) onLoad(fig,b.Value);
st.rnd.ValueChangedFcn = @(~,~) onRandom(fig);
for i = 1:nS
    st.cb(i).ValueChangedFcn = @(~,~) updateLayout(fig);
end
st.stop.ButtonPushedFcn = @(~,~) closePanel(fig);
fig.CloseRequestFcn = @(~,~) closePanel(fig);

tm = timer('ExecutionMode','fixedSpacing', ...
    'Period',0.1, ...
    'BusyMode','drop', ...
    'TimerFcn',@(~,~) onTick(fig));
ud = fig.UserData;
ud.timer = tm;
fig.UserData = ud;

%% Configure continuous real-time simulation

% Run indefinitely until the user presses Stop and close.
set_param(mdl,'StopTime','inf');

% Synchronize simulation time with real wall-clock time.
set_param(mdl,'EnablePacing','on');
set_param(mdl,'PacingRate','1');

% Display the active configuration.
fprintf('\nLIVE SIMULATION SETTINGS\n');
fprintf('StopTime:     %s\n',get_param(mdl,'StopTime'));
fprintf('EnablePacing: %s\n',get_param(mdl,'EnablePacing'));
fprintf('PacingRate:   %s\n',get_param(mdl,'PacingRate'));

% Start live simulation and plotting timer.
set_param(mdl,'SimulationCommand','start');
start(tm);
if nargout == 0
    clear fig
end
end

% -------------------------------------------------------------------------
function updateLayout(fig)
% Show only ticked signals; collapse axes without ticked signals.
ud = fig.UserData;
nA = numel(ud.ax);
h = cell(1,nA);
for a = 1:nA
    idx = find([ud.sig{:,3}] == a);
    on = any(arrayfun(@(i) ud.cb(i).Value,idx));
    if on
        if a == 1
            h{a} = '2x';
        else
            h{a} = '1x';
        end
    else
        h{a} = 0;
    end
    ud.ax(a).Visible = on;
    set(ud.ax(a).Children,'Visible',on);
end
for i = 1:size(ud.sig,1)
    if ud.cb(i).Value
        ud.ln(i).Visible = 'on';
    else
        ud.ln(i).Visible = 'off';
    end
end
ud.Rg.RowHeight = h;
end

function onRef(fig,v)
if ~isvalid(fig)
    return
end
ud = fig.UserData;
v = round(v);
set_param([ud.model '/ref_w'],'Value',num2str(v));
ud.refT.Text = sprintf('Reference speed: %d rad/s',v);
end

function onLoad(fig,on)
ud = fig.UserData;
set_param([ud.model '/load_on'],'Value',num2str(double(on)));
if on
    ud.load.Text = 'Load ON (click to switch off)';
else
    ud.load.Text = 'Load OFF (click to switch on)';
end
end

function onRandom(fig)
ud = fig.UserData;
if ud.rnd.Value
    ud.load.Enable = 'off';
    ts = get_param(ud.model, 'SimulationTime');
    ud.next = ts + 2 + 4*rand;
    fig.UserData = ud;
else
    ud.load.Enable = 'on';
end
end

function onTick(fig)

if ~isvalid(fig)
    return
end

ud = fig.UserData;
mdl = ud.model;

if ~bdIsLoaded(mdl) || ...
        ~strcmp(get_param(mdl,'SimulationStatus'),'running')
    ud.info.Text = 'simulation not running';
    return
end

ts = get_param(ud.model, 'SimulationTime');

% ---------------------------------------------------------
% Random load switching
% ---------------------------------------------------------

if ud.rnd.Value && ts >= ud.next

    on = ~ud.load.Value;
    ud.load.Value = on;
    onLoad(fig,on);

    d = ud.rand_on;
    if ~on
        d = ud.rand_off;
    end

    ud = fig.UserData;
    ud.next = ts + d(1) + (d(2)-d(1))*rand;
    fig.UserData = ud;
end

% ---------------------------------------------------------
% Read original plant, limits, reference and load signals
% ---------------------------------------------------------

try
    rp = get_param([mdl '/Plant'],'RuntimeObject');
    rl = get_param([mdl '/limits'],'RuntimeObject');
    rr = get_param([mdl '/ref_w'],'RuntimeObject');
    ro = get_param([mdl '/load_on'],'RuntimeObject');

    w = double(rp.OutputPort(1).Data);
    u = double(rl.OutputPort(1).Data);
    r = double(rr.OutputPort(1).Data);
    on = double(ro.OutputPort(1).Data);

catch ME
    fprintf('\n[PLANT READ ERROR] t = %.3f s\n%s\n', ...
        ts,ME.message);

    ud.info.Text = ['Plant signal read error: ' ME.message];
    return
end

% ---------------------------------------------------------
% Observer debugging
% ---------------------------------------------------------

% Persistent variables prevent flooding the command window.
persistent lastPrintTime lastObserverError

if isempty(lastPrintTime)
    lastPrintTime = -inf;
end

% Reset debugging timer when a new simulation starts.
if ts < lastPrintTime
    lastPrintTime = -inf;
    lastObserverError = '';
end

observerOK = false;

x_hat_deviation = [];
omega_e_obs = NaN;
omega_e_meas = NaN;
e_obs = NaN;

try

    % Actual source outputs verified in Simulink:
    % Gain1 -> omega_e_obs Outport
    % Plant -> omega_e_meas Outport

    %% Read observer deviation states and measured physical speed

    % Read the signal feeding Sum1 input 1. The known model wiring is:
    % Observer output -> Sum1 input 1, Constant1 -> Sum1 input 2.
    % This reads the raw Observer output BEFORE the +ones offset and Gain1.
    % We trace the Sum1 input connection because direct access to the
    % StateSpace RuntimeObject by block path is not supported reliably here.
    x_hat_deviation = readInputSourceValue(mdl,'Sum1',1);
    x_hat_deviation = x_hat_deviation(:);

    % Actual measured engine speed in rad/s.
    omega_e_meas = readOutportSourceValue(mdl,'omega_e_meas');

    % Check observer state vector dimension.
    if numel(x_hat_deviation) ~= size(ud.Obs_Cy,2)
        error('Observer has %d states, Obs_Cy expects %d.', ...
            numel(x_hat_deviation),size(ud.Obs_Cy,2));
    end

    % Check measurement.
    if ~isscalar(omega_e_meas)
        error('omega_e_meas is not scalar.');
    end

    % Obs_Cy maps raw observer deviation states to normalized speed
    % deviation. The +1 restores the nominal operating point.
    omega_normalized_deviation = ud.Obs_Cy * x_hat_deviation;
    omega_e_obs = ud.ISCS_Ty * (1 + omega_normalized_deviation);
    e_obs = omega_e_obs - omega_e_meas;
    observerOK = true;

    % Print recovery message if observer reading previously failed.
    if ~isempty(lastObserverError)
        fprintf('\n[OBSERVER RECOVERED] t = %.3f s\n',ts);
        lastObserverError = '';
    end

    % Print actual observer data once per second.
    if ts - lastPrintTime >= 1

        fprintf('\n');
        fprintf('============================================\n');
        fprintf('[OBSERVER DEBUG] t = %.3f s\n',ts);
        fprintf('============================================\n');

        fprintf('Raw observer deviation states (%d):\n',numel(x_hat_deviation));
        disp(x_hat_deviation);
        %% DEBUG: Contribution of each observer state

        contributions = ud.Obs_Cy(:).*x_hat_deviation(:);

        Tobs = table( ...
            (1:numel(x_hat_deviation))', ...
            ud.Obs_Cy(:), ...
            x_hat_deviation(:), ...
            contributions, ...
            'VariableNames', ...
            {'State','Obs_Cy','x_hat_deviation','Contribution'});

        disp('Contribution of individual observer states:');
        disp(Tobs);

        fprintf('Sum of contributions: %.10f\n', ...
            sum(contributions));

        fprintf('Obs_Cy * x_hat_deviation = %.10f\n', ...
            omega_normalized_deviation);
        fprintf('ISCS_Ty        = %.10f\n',ud.ISCS_Ty);

        fprintf('omega_e_obs    = %.8f rad/s\n',omega_e_obs);
        fprintf('omega_e_meas   = %.8f rad/s\n',omega_e_meas);

        fprintf('Observer error = %.8f rad/s\n',e_obs);
        fprintf('Load: %s | P_load: %.0f W\n', ...
            ternary(on ~= 0,'ON','OFF'),on*ud.Pon);
        
        fprintf('Plant w        = %.8f rad/s\n',w);
        fprintf('Plant difference = %.8f rad/s\n', ...
            w-omega_e_meas);

        fprintf('============================================\n');

        lastPrintTime = ts;
    end

catch ME

    % Print each distinct error only once.
    if isempty(lastObserverError) || ...
            ~strcmp(lastObserverError,ME.message)

        fprintf('\n[OBSERVER READ ERROR] t = %.3f s\n',ts);
        fprintf('%s\n',ME.message);

        lastObserverError = ME.message;
    end

end

% ---------------------------------------------------------
% Assemble all original plot signals
% ---------------------------------------------------------

% e_obs is NaN only if observer reading failed.
% All other signals are still recorded.

val = [w, r, r-w, e_obs, u(1), u(2), ...
    double(rp.OutputPort(2).Data)/1000, ...
    double(rp.OutputPort(3).Data), ...
    double(rp.OutputPort(4).Data), ...
    1000*double(rp.OutputPort(5).Data), ...
    1000*double(rp.OutputPort(6).Data), ...
    on*ud.Pon];

% ---------------------------------------------------------
% Plot all available signals
% ---------------------------------------------------------

for i = 1:numel(val)

    if isfinite(val(i))
        addpoints(ud.ln(i),ts,val(i));
    end

end

% ---------------------------------------------------------
% Automatic graph scaling
% ---------------------------------------------------------

x = [max(0,ts-ud.win), max(ts,ud.win)];

for a = 1:numel(ud.ax)

    if ud.ax(a).Visible ~= "on"
        continue
    end

    xlim(ud.ax(a),x);

    ymin = inf;
    ymax = -inf;

    for i = find([ud.sig{:,3}] == a)

        if ~ud.cb(i).Value
            continue
        end

        [px,py] = getpoints(ud.ln(i));
        py = py(px >= x(1));

        if ~isempty(py)
            ymin = min(ymin,min(py));
            ymax = max(ymax,max(py));
        end

    end

    if isinf(ymin)
        continue
    end

    lo = min(0,ymin);
    hi = max(0,ymax);

    pad = 0.08*max(hi-lo,1e-3);

    if lo < 0
        lo = lo-pad;
    end

    ylim(ud.ax(a),[lo,hi+pad]);

end

% ---------------------------------------------------------
% Status line
% ---------------------------------------------------------

if w < 50
    flag = '   *** STALL: speed below 50 rad/s ***';
else
    flag = '';
end

if observerOK

    obsText = sprintf( ...
        'observer %.2f | measured %.2f | observer error %.4f rad/s | ', ...
        omega_e_obs,omega_e_meas,e_obs);

else

    obsText = 'observer unavailable - see Command Window | ';

end

ud.info.Text = sprintf([ ...
    't = %.1f s | reference %.0f | speed %.2f rad/s | ' ...
    '%sthrottle %.2f %% | load %s%s'], ...
    ts,r,w,obsText,u(1), ...
    ternary(on ~= 0,'ON','off'),flag);

end

function y = readOutportSourceValue(mdl,outportName)
% Read the live value at the source port connected to a top-level Outport.
% This follows the same route verified in the MATLAB debugging commands.

outportBlock = [mdl '/' outportName];
if getSimulinkBlockHandle(outportBlock) == -1
    error('Outport block "%s" was not found.',outportBlock);
end

ph = get_param(outportBlock,'PortHandles');
if isempty(ph.Inport)
    error('Outport "%s" has no input port.',outportBlock);
end

lineHandle = get_param(ph.Inport(1),'Line');
if isempty(lineHandle) || lineHandle == -1
    error('Outport "%s" is not connected.',outportBlock);
end

srcPort = get_param(lineHandle,'SrcPortHandle');
srcBlock = get_param(lineHandle,'SrcBlockHandle');
if isempty(srcBlock) || srcBlock == -1 || isempty(srcPort) || srcPort == -1
    error('Could not identify the source of Outport "%s".',outportBlock);
end

ports = get_param(srcBlock,'PortHandles');
portIndex = find(ports.Outport == srcPort,1,'first');
if isempty(portIndex)
    error('Source output port for "%s" could not be identified.',outportName);
end

rt = get_param(srcBlock,'RuntimeObject');
if isempty(rt)
    error('RuntimeObject of source block "%s" is empty. Is the simulation running?', ...
        getfullname(srcBlock));
end

y = double(rt.OutputPort(portIndex).Data);
if isempty(y) || any(~isfinite(y(:)))
    error('Source signal "%s" contains empty or non-finite values.',outportName);
end
end

function closePanel(fig)
if ~isvalid(fig)
    return
end
ud = fig.UserData;
try
    stop(ud.timer);
    delete(ud.timer);
catch
end
try
    if bdIsLoaded(ud.model)
        set_param(ud.model,'SimulationCommand','stop');
    end
catch
end
% Deliberately do NOT close the Simulink model or its window.
delete(fig);
end

function s = ternary(c,a,b)
if c
    s = a;
else
    s = b;
end
end

function n = nm_of(f)
[~,n] = fileparts(f);
end

function m = getMain()
% Run main.m inside this function so its clear does not affect the caller.
main
m = struct('constants',constants, ...
    'limits',limits, ...
    'simcfg',simcfg, ...
    'init',init, ...
    'data',data);
end

function projectRoot = findProjectRoot(startFolder)
% Find the folder that contains M1 Model and M4 Competition.
projectRoot = startFolder;
while true
    if isfolder(fullfile(projectRoot,'M1 Model')) && ...
       isfolder(fullfile(projectRoot,'M4 Competition'))
        return
    end
    parentFolder = fileparts(projectRoot);
    if isempty(parentFolder) || strcmp(parentFolder,projectRoot)
        error('Could not locate project root containing M1 Model and M4 Competition.');
    end
    projectRoot = parentFolder;
end
end
function y = readInputSourceValue(mdl,blockName,inputIndex)
% Read the live signal feeding one input port of a block.
% This follows the input line to its source block/port and then reads the
% source RuntimeObject through its block handle. This is the same access
% route that successfully read Observer -> Sum1 input 1 during diagnosis.

targetBlock = [mdl '/' blockName];
if getSimulinkBlockHandle(targetBlock) == -1
    error('Block "%s" was not found.',targetBlock);
end

ph = get_param(targetBlock,'PortHandles');
if inputIndex < 1 || inputIndex > numel(ph.Inport)
    error('Block "%s" has no input port %d.',targetBlock,inputIndex);
end

lineHandle = get_param(ph.Inport(inputIndex),'Line');
if isempty(lineHandle) || lineHandle == -1
    error('Input port %d of block "%s" is not connected.', ...
        inputIndex,targetBlock);
end

srcPort = get_param(lineHandle,'SrcPortHandle');
srcBlock = get_param(lineHandle,'SrcBlockHandle');
if isempty(srcBlock) || srcBlock == -1 || isempty(srcPort) || srcPort == -1
    error('Could not identify the source connected to %s input %d.', ...
        targetBlock,inputIndex);
end

ports = get_param(srcBlock,'PortHandles');
portIndex = find(ports.Outport == srcPort,1,'first');
if isempty(portIndex)
    error('Could not identify the source output port for %s input %d.', ...
        targetBlock,inputIndex);
end

% Use the source block handle (not its path string). This access pattern
% was confirmed to work for the Observer StateSpace block in this model.
rt = get_param(srcBlock,'RuntimeObject');
if isempty(rt)
    error('RuntimeObject of source block "%s" is empty.', ...
        getfullname(srcBlock));
end

y = double(rt.OutputPort(portIndex).Data);
if isempty(y) || any(~isfinite(y(:)))
    error('Signal from "%s" to %s input %d is empty or non-finite.', ...
        getfullname(srcBlock),targetBlock,inputIndex);
end
end
