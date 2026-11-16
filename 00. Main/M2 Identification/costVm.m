function e = costVm(Vm, in, ident, data)
% Manual p.28: e = sum_i (p_m(i) - p_m_bar(i))^2 for the intake manifold volume Vm [m^3].
ident.Vm = Vm;
out = sim(in.setVariable('ident', ident));
s   = out.logsout.get('p_m').Values;
pm_sim  = interp1(s.Time, s.Data(:), data.t);
pm_meas = data.meas.p_m.signals.values(:);
e = sum((pm_sim - pm_meas).^2);
end
