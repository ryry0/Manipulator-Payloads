%This is a utility file to get average Simulation data
num_trials = 100;

mass_sum = 0;
inertia_sum = 0;
ctr_sum = 0;
mass_err_sum = 0;
inertia_err_sum = 0;
ctr_err_sum = 0;

for count = 1:num_trials
	Simulation_Error
	mass_sum = mass_sum + m2_hat;
	ctr_sum = ctr_sum + lc2_hat;
	inertia_sum = inertia_sum + I2_hat;
	mass_err_sum = mass_err_sum + m2_percent_error;
	ctr_err_sum = ctr_err_sum + lc2_percent_error;
	inertia_err_sum = inertia_err_sum + I2_percent_error;
end

mass_avg = mass_sum/num_trials
ctr_avg = ctr_sum/num_trials
inertia_avg = inertia_sum/num_trials

mass_err_avg = mass_err_sum/num_trials
ctr_err_avg = ctr_err_sum/num_trials
inertia_err_avg = inertia_err_sum/num_trials
