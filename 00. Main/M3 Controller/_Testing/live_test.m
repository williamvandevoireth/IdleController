function fig = live_test(ctrlFile, opt)
%LIVE_TEST  Interactive test: one window with live graphs on the right and sliders, buttons and tick boxes on the left.
%
%   live_test('controller1_William.mat')
%   live_test(file, P_on=1200, rand_on=[4 12], rand_off=[4 12], du_min=-30, window=40)
%
%   Runs Bench_Live.slx in real time (pacing 1x) until you press "Stop and close" or close the window.
%   Controls (left):
%     Reference slider   70-200 rad/s: the speed the controller should hold. The controller sets throttle and ignition by itself.
%     Load button        electrical load (water boiler, P_on W) on / off
%     Random load        switches the load on and off at random times (durations from rand_on / rand_off seconds); the button is then locked
%     Tick boxes         choose what the graphs show: speed, reference, speed error, throttle, ignition, manifold pressure, engine torque,
%                        load torque, air mass flows, load power. A graph with no ticked signal disappears. Data of unticked signals keeps
%                        being recorded, so you can tick them later and see the history of the sliding window.
%   The speed measurement carries the noise recorded on the test bench. The plant is Engine_Plant_Live.slx (bench-identified parameters).
%   Hint: below about 85 rad/s a load step can stall the model (speed < 50 rad/s is marked in the status line).
%   Observer error: if the controller comes with its observer matrices (Aobs, Bobs, Cobs, Dobs, Obs_Cy, saved in the controller file or
%   in <name>_observer.mat), the window uses Observer_Test_Live.slx and adds a graph with e_obs = omega_e,obs - omega_e,meas.
%   Offline tests with the cost J: run_test.m / test_controller.m.
arguments
    ctrlFile
    opt.P_on     (1,1) double = 1200
    opt.rand_on  (1,2) double = [4 12]
    opt.rand_off (1,2) double = [4 12]
    opt.du_min   (1,1) double = -30
    opt.window   (1,1) double = 40
end
here = fileparts(mfilename('fullpath'));
pd = fullfile(fileparts(fileparts(here)), {'M1 Model','M2 Identification','M3 Controller','M4 Competition'});   pd = [pd, fullfile(pd,'functions'), fullfile(pd,'models'), fullfile(pd,'_Models'), fullfile(pd,'_Testing')];   addpath(strjoin(pd(isfolder(pd)), pathsep));   % milestone folders with their functions and models

%% controller file (a bare name is searched in the personal folders of M3 Controller)
ctrlFile = char(ctrlFile);
if ~isfile(ctrlFile) && isfile(which(ctrlFile)), ctrlFile = which(ctrlFile); end
if ~isfile(ctrlFile)
    [pp, nm, ex] = fileparts(ctrlFile);
    if isempty(pp)
        hit = dir(fullfile(fileparts(fileparts(here)), 'M3 Controller', '*', [nm ex]));
        if numel(hit) == 1, ctrlFile = fullfile(hit.folder, hit.name); end
    end
end
if ~isfile(ctrlFile), error('Controller file "%s" not found.', ctrlFile); end
c = load(ctrlFile);
need = {'ISCS_Ad',[12 12]; 'ISCS_Bd',[12 1]; 'ISCS_Cd',[2 12]; 'ISCS_Dd',[2 1]; 'ISCS_Tu',[2 2]; 'ISCS_Ty',[1 1]};
for i = 1:size(need,1)
    if ~isfield(c, need{i,1}) || ~isequal(size(c.(need{i,1})), need{i,2}), error('%s: variable %s missing or not %s.', ctrlFile, need{i,1}, mat2str(need{i,2})); end
end

%% optional observer data: Aobs, Bobs, Cobs, Dobs, Obs_Cy (saved by the design script in the controller file or in <name>_observer.mat beside it)
% With it the window runs Observer_Test_Live.slx (Bench_Live plus a copy of the observer) and shows the observer error
% e_obs = omega_e_obs - omega_e_meas; without it the plain Bench_Live.slx is used.
obsNames = {'Aobs','Bobs','Cobs','Dobs','Obs_Cy'};   O = c;
obsFile = fullfile(fileparts(ctrlFile), [nm_of(ctrlFile) '_observer.mat']);
if ~all(isfield(O, obsNames)) && isfile(obsFile), O = load(obsFile); end
hasObs = all(isfield(O, obsNames));
if hasObs && ~isequal(size(O.Obs_Cy), [1 size(c.ISCS_Ad,1)-1]), error('Obs_Cy must be 1x%d (observer states of the controller).', size(c.ISCS_Ad,1)-1); end

%% workspace of the models (same set-up as test_controller.m)
m  = getMain();                                   % constants, limits, simcfg, init, data as built by main.m
S  = load('LinModels_bench.mat');   M = S.M;   Lm = load('LinModel_bench.mat');
sameDu = find([M.delta_u_nom] == c.ISCS_Tu(2,2));   if isempty(sameDu), sameDu = 1:numel(M); end
[~, j] = min(abs([M(sameDu).omega_e_nom] - c.ISCS_Ty));   k = sameDu(j);
init = m.init;   init.omega_e0 = c.ISCS_Ty;   init.p_m0 = M(k).p_m_nom;   init.du_init = c.ISCS_Tu(2,2);   init.q_init = M(k).Q_nom;
simcfg = m.simcfg;   simcfg.omega_meas = 0;
data = m.data;   t = data.meas.time(:);   N = numel(t);   c1 = ones(N,1);
data.meas.T_a.signals.values = Lm.theta_a_nom*c1;   data.meas.p_a.signals.values = Lm.p_a_nom*c1;
data.meas.T_m.signals.values = Lm.theta_m_nom*c1;   data.meas.p_e.signals.values = Lm.p_e_nom*c1;
q  = load(fullfile(fileparts(which('main')), '..', '..', 'ISCSData', 'studentId_0002.mat')).meas;
wq = q.omega_e.signals.values(:);   noise = wq(1:N) - movmean(wq(1:N), 200);      % recorded high-frequency speed noise, repeated cyclically
V = struct('data',data, 'init',init, 'ident',S.ident, 'constants',m.constants, 'limits',m.limits, 'simcfg',simcfg, ...
           'ISCS_Ad',c.ISCS_Ad, 'ISCS_Bd',c.ISCS_Bd, 'ISCS_Cd',c.ISCS_Cd, 'ISCS_Dd',c.ISCS_Dd, 'ISCS_Tu',c.ISCS_Tu, 'ISCS_Ty',c.ISCS_Ty, ...
           'Ts',1e-3, 'noise_ts',struct('time',[], 'signals',struct('values',noise, 'dimensions',1)), 'du_min',opt.du_min, 'P_on',opt.P_on);   % empty time vector: cyclic repetition
f = fieldnames(V);   for i = 1:numel(f), assignin('base', f{i}, V.(f{i})); end
if hasObs, for i = 1:numel(obsNames), assignin('base', obsNames{i}, O.(obsNames{i})); end, end

%% model
mdl = 'Bench_Live';   if hasObs, mdl = 'Observer_Test_Live'; end
if bdIsLoaded(mdl), set_param(mdl,'SimulationCommand','stop'); close_system(mdl,0); end
load_system(mdl);
w0 = min(max(round(c.ISCS_Ty), 70), 200);
set_param([mdl '/ref_w'], 'Value', num2str(w0));   set_param([mdl '/load_on'], 'Value', '0');

%% signals that can be shown:  key, label, graph (axis), line colour, shown at start
sig = { 'speed', 'speed \omega_e',            1, [0.85 0.10 0.10], true
        'ref',   'reference',                 1, [0 0 0],          true
        'err',   'speed error (ref - speed)', 1, [0.93 0.69 0.13], false
        'ua',    'throttle u_\alpha [%]',     2, [0 0.45 0.74],    true
        'du',    'ignition \Delta u_\zeta',   3, [0.47 0.67 0.19], true
        'pm',    'manifold pressure p_m',     4, [0.49 0.18 0.56], false
        'Te',    'engine torque T_e',         5, [0.85 0.33 0.10], false
        'Tl',    'load torque T_l',           5, [0.30 0.75 0.93], false
        'mda',   'air flow throttle',         6, [0 0.45 0.74],    false
        'mdb',   'air flow engine',           6, [0.85 0.33 0.10], false
        'Pl',    'load power P_l',            7, [0.5 0.5 0.5],    true };
axTitle = {'Speed [rad/s]', 'Throttle [%]', 'Ignition retardation [degCA]', 'Manifold pressure [kPa]', 'Torque [Nm]', 'Air mass flow [g/s]', 'Load power [W]'};
if hasObs        % extra graph with the observer error, below the speed graph's signals
    sig = [sig(1:3,:); {'eobs', 'observer error (\omega_{e,obs} - \omega_{e,meas})', 8, [0 0.45 0.74], true}; sig(4:end,:)];
    axTitle{end+1} = 'Observer error [rad/s]';
end
nS = size(sig,1);   nA = numel(axTitle);

%% window: controls left, graphs right
obsCy = [];   if hasObs, obsCy = O.Obs_Cy; end
st = struct('model',mdl, 'rand_on',opt.rand_on, 'rand_off',opt.rand_off, 'next',inf, 'win',opt.window, 'du_nom',c.ISCS_Tu(2,2), 'Pon',opt.P_on, 'sig',{sig}, 'axTitle',{axTitle}, 'hasObs',hasObs, 'Obs_Cy',obsCy, 'ISCS_Ty',c.ISCS_Ty);
fig = uifigure('Name', ['live_test: ' nm_of(ctrlFile)], 'Position', [40 40 1200 700]);
G = uigridlayout(fig, [1 2]);   G.ColumnWidth = {310, '1x'};   G.Padding = [8 8 8 8];
L = uigridlayout(G, [7 1]);   L.RowHeight = {22, 40, 40, 26, '1x', 54, 34};   L.Padding = [2 2 2 2];   L.RowSpacing = 6;
st.refT = uilabel(L, 'Text', sprintf('Reference speed: %d rad/s', w0), 'FontWeight', 'bold');
st.ref  = uislider(L, 'Limits', [70 200], 'Value', w0, 'MajorTicks', 70:30:190);
st.load = uibutton(L, 'state', 'Text', 'Load OFF (click to switch on)', 'FontSize', 14);
st.rnd  = uicheckbox(L, 'Text', sprintf('Random load (on %g-%g s, off %g-%g s)', opt.rand_on, opt.rand_off));
pn = uipanel(L, 'Title', 'Show in graphs');
pg = uigridlayout(pn, [nS 1]);   pg.RowHeight = repmat({20}, 1, nS);   pg.Padding = [6 2 2 2];   pg.RowSpacing = 1;
st.cb = gobjects(1, nS);
for i = 1:nS, st.cb(i) = uicheckbox(pg, 'Text', sig{i,2}, 'Value', sig{i,5}); end
st.info = uilabel(L, 'Text', 'starting ...', 'WordWrap', 'on', 'VerticalAlignment', 'top');
st.stop = uibutton(L, 'push', 'Text', 'Stop and close', 'BackgroundColor', [0.85 0.3 0.3], 'FontColor', 'w');

st.Rg = uigridlayout(G, [nA 1]);   st.Rg.RowSpacing = 4;   st.Rg.Padding = [2 2 2 2];
st.ax = gobjects(1, nA);
for a = 1:nA
    st.ax(a) = uiaxes(st.Rg);   grid(st.ax(a), 'on');   title(st.ax(a), axTitle{a}, 'FontSize', 10);   st.ax(a).FontSize = 9;
end
xlabel(st.ax(nA), 't [s]');   title(st.ax(1), [axTitle{1} '  -  ' strrep(nm_of(ctrlFile), '_', '\_')], 'FontSize', 10);
st.ln = gobjects(1, nS);
for i = 1:nS
    st.ln(i) = animatedline(st.ax(sig{i,3}), 'Color', sig{i,4}, 'DisplayName', sig{i,2}, 'LineWidth', 1.2, 'MaximumNumPoints', 20000);
end
for a = [1 5 6], legend(st.ax(a), 'Location', 'northeast', 'FontSize', 8, 'Box', 'off'); end
fig.UserData = st;
updateLayout(fig);

st.ref.ValueChangingFcn  = @(~, ev) onRef(fig, ev.Value);   st.ref.ValueChangedFcn = @(s, ~) onRef(fig, s.Value);
st.load.ValueChangedFcn  = @(b, ~) onLoad(fig, b.Value);
st.rnd.ValueChangedFcn   = @(~, ~) onRandom(fig);
for i = 1:nS, st.cb(i).ValueChangedFcn = @(~, ~) updateLayout(fig); end
st.stop.ButtonPushedFcn  = @(~, ~) closePanel(fig);
fig.CloseRequestFcn      = @(~, ~) closePanel(fig);

tm = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.1, 'BusyMode', 'drop', 'TimerFcn', @(tmr, ~) onTick(fig, tmr));
ud = fig.UserData;   ud.timer = tm;   fig.UserData = ud;
set_param(mdl, 'SimulationCommand', 'start');             % real time (pacing 1x)
start(tm);
if nargout == 0, clear fig, end
end

% ---------------------------------------------------------------------------------------------------------------------
function updateLayout(fig)
% show only the ticked signals; a graph without a ticked signal collapses
ud = fig.UserData;   nA = numel(ud.ax);   h = cell(1, nA);
for a = 1:nA
    idx = find([ud.sig{:,3}] == a);   on = any(arrayfun(@(i) ud.cb(i).Value, idx));
    if on, if a == 1, h{a} = '2x'; else, h{a} = '1x'; end, else, h{a} = 0; end
    ud.ax(a).Visible = on;   set(ud.ax(a).Children, 'Visible', on);
end
for i = 1:size(ud.sig,1)
    if ud.cb(i).Value, ud.ln(i).Visible = 'on'; else, ud.ln(i).Visible = 'off'; end
end
ud.Rg.RowHeight = h;
end

function onRef(fig, v)
if ~isvalid(fig), return, end
ud = fig.UserData;   v = round(v);
set_param([ud.model '/ref_w'], 'Value', num2str(v));
ud.refT.Text = sprintf('Reference speed: %d rad/s', v);
end

function onLoad(fig, on)
ud = fig.UserData;
set_param([ud.model '/load_on'], 'Value', num2str(double(on)));
if on, ud.load.Text = 'Load ON (click to switch off)'; else, ud.load.Text = 'Load OFF (click to switch on)'; end
end

function onRandom(fig)
ud = fig.UserData;
if ud.rnd.Value
    ud.load.Enable = 'off';
    ud.next = get_param(ud.model, 'SimulationTime') + 2 + 4*rand;      % first switch after 2-6 s
    fig.UserData = ud;
else
    ud.load.Enable = 'on';
end
end

function onTick(fig, tmr)
if ~isvalid(fig)                                   % window gone (closed in another way): stop the simulation and remove this timer
    for mm = {'Bench_Live', 'Observer_Test_Live'}, try, if bdIsLoaded(mm{1}), set_param(mm{1}, 'SimulationCommand', 'stop'); end, catch, end, end
    stop(tmr);   delete(tmr);   return
end
ud = fig.UserData;   mdl = ud.model;
if ~bdIsLoaded(mdl) || ~strcmp(get_param(mdl, 'SimulationStatus'), 'running'), ud.info.Text = 'simulation not running'; return, end
ts = get_param(mdl, 'SimulationTime');
% random load switching (simulation time)
if ud.rnd.Value && ts >= ud.next
    on = ~ud.load.Value;   ud.load.Value = on;   onLoad(fig, on);
    d = ud.rand_on;   if ~on, d = ud.rand_off; end
    ud = fig.UserData;   ud.next = ts + d(1) + (d(2) - d(1))*rand;   fig.UserData = ud;
end
% read the signals of the running model
try
    rp = get_param([mdl '/Plant'],   'RuntimeObject');   rl = get_param([mdl '/limits'],  'RuntimeObject');
    rr = get_param([mdl '/ref_w'],   'RuntimeObject');   ro = get_param([mdl '/load_on'], 'RuntimeObject');
    w = double(rp.OutputPort(1).Data);   u = double(rl.OutputPort(1).Data);   r = double(rr.OutputPort(1).Data);   on = double(ro.OutputPort(1).Data);
    val = [w, r, r - w, u(1), u(2), double(rp.OutputPort(2).Data)/1000, double(rp.OutputPort(3).Data), double(rp.OutputPort(4).Data), ...
           1000*double(rp.OutputPort(5).Data), 1000*double(rp.OutputPort(6).Data), on*ud.Pon];
catch
    return
end
e_obs = NaN;
if ud.hasObs                      % observer estimate of the speed minus the measured speed (NaN if it cannot be read)
    try
        xh = readInputSourceValue(mdl, 'Sum1', 1);                      % raw observer states (deviation), before the +1 offset
        e_obs = ud.ISCS_Ty*(1 + ud.Obs_Cy*xh(:)) - readOutportSourceValue(mdl, 'omega_e_meas');
    catch
    end
    val = [val(1:3), e_obs, val(4:end)];
end
for i = 1:numel(val), if isfinite(val(i)), addpoints(ud.ln(i), ts, val(i)); end, end
x = [max(0, ts - ud.win), max(ts, ud.win)];
for a = 1:numel(ud.ax)
    if ud.ax(a).Visible ~= "on", continue, end
    xlim(ud.ax(a), x);
    % y-axis: always include 0; otherwise fit the ticked signals within the time window
    ymin = inf;   ymax = -inf;
    for i = find([ud.sig{:,3}] == a)
        if ~ud.cb(i).Value, continue, end
        [px, py] = getpoints(ud.ln(i));   py = py(px >= x(1));
        if ~isempty(py), ymin = min(ymin, min(py));   ymax = max(ymax, max(py)); end
    end
    if isinf(ymin), continue, end
    lo = min(0, ymin);   hi = max(0, ymax);   pad = 0.08*max(hi - lo, 1e-3);
    if lo < 0, lo = lo - pad; end
    ylim(ud.ax(a), [lo, hi + pad]);
end
if w < 50, flag = '   *** STALL: speed below 50 rad/s ***'; else, flag = ''; end
obsTxt = '';   if ud.hasObs, obsTxt = sprintf(' | observer error %.2f rad/s', e_obs); end
ud.info.Text = sprintf('t = %.0f s | reference %.0f | speed %.0f rad/s%s | throttle %.1f %% | load %s%s', ts, r, w, obsTxt, u(1), ternary(on ~= 0, 'ON', 'off'), flag);
end

function closePanel(fig)
if ~isvalid(fig), return, end
ud = fig.UserData;
try, stop(ud.timer); delete(ud.timer); catch, end
try, if bdIsLoaded(ud.model), set_param(ud.model, 'SimulationCommand', 'stop'); end, catch, end
delete(fig);
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

function n = nm_of(f)
[~, n] = fileparts(f);
end

function m = getMain()
% runs main.m inside this function, so its 'clear' does not touch the caller's or the base workspace
main
m = struct('constants',constants, 'limits',limits, 'simcfg',simcfg, 'init',init, 'data',data);
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
