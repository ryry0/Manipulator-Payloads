%this script will be used to optimize the Simulation in different parameters


mass_error = double.empty;
ctr_err = double.empty;
inertia_err = double.empty;
condition_vector = double.empty;

for y_final = -.673:.001:-.475
 
    e     
    %mass_error(i) = m2_percent_error;
    condition_vector(i) = cond(Y,2);
    %inertia_err(i) = I2_percent_error;
	%mass_error = [mass_error m2_percent_error];
	%ctr_err = [ctr_err lc2_percent_error];
	%inertia_err = [inertia_err I2_percent_error];
    %condition_vector = [condition_vector cond(Y,2)];%sqrt(trace(transpose(Y)*Y)*trace(inv(transpose(Y)*Y)))];
%end
end

[~,condition_min] = find(condition_vector==min(min(condition_vector)))
[~,mass_error_min] = find(mass_error==min(min(mass_error)))
[~,ctr_err_min] = find(ctr_err==min(min(ctr_err)))
[~,inertia_err_min] = find(inertia_err==min(min(inertia_err)))
