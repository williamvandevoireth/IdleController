function e = costTorque(x, in, ident, data, p0)
% Normalized omega_e + p_m error for p = x.*p0 = [eta0 eta1 beta0 Theta_e].
p = x.*p0;
ident.eta0 = p(1); ident.eta1 = p(2); ident.beta0 = p(3); ident.Theta_e = p(4);
out = sim(in.setVariable('ident', ident));
sw = out.logsout.get('omega_e').Values;  sp = out.logsout.get('p_m').Values;
w_sim  = interp1(sw.Time, sw.Data(:), data.t);
pm_sim = interp1(sp.Time, sp.Data(:), data.t);
w_meas  = data.meas.omega_e.signals.values(:);
pm_meas = data.meas.p_m.signals.values(:);
e = sum((w_sim-w_meas).^2)/var(w_meas) + sum((pm_sim-pm_meas).^2)/var(pm_meas);
end