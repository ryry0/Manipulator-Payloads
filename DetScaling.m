% This script determines the best scaling for a particular
% set of data.
mass_error = double.empty;
ctr_err = double.empty;
inertia_err = double.empty;

for scaling_factor = 1:.01:2
	AnalyzeData
	mass_error = [mass_error m2_percent_error];
	ctr_err = [ctr_err lc2_percent_error];
	inertia_err = [inertia_err I2_percent_error];
end

[~,mass_error_min] = find(mass_error==min(min(mass_error)))
[~,ctr_err_min] = find(ctr_err==min(min(ctr_err)))
[~,inertia_err_min] = find(inertia_err==min(min(inertia_err)))
