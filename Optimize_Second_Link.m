%%this script will be used to optimize the Simulation in different parameters


mass_error = double.empty;
ctr_err = double.empty;
inertia_err = double.empty;
condition_vector = double.empty;

t_final = 2;
%q_final = .001:.001:pi/6;

i = 1;
freq = .25:.01:3;
%data = datasecondlinkloaded5lbexp1(1:2000,:);
%%
for q_final = .001:.001:pi/6;
	q_traj = [0,0,0,0; 0, 0, 0, t_final];

	 q2_traj = [0, 0, 0, 0;
	 q_final, 0, 0, t_final];
 
 [   q1_trajectory, q1_velocity, q1_acceleration, ...
     q2_trajectory, q2_velocity, q2_acceleration] = ...
     GenArbJointTraj(q_traj, q2_traj, delta_t, l1, l2);

    Sim_Arb_Client;
    A = [Y(:,2) Y(:,3) Y(:,5)];    
    %mass_error(i) = m2_percent_error;
    condition_vector(i) = cond(A,2);
    %inertia_err(i) = I2_percent_error;
	%mass_error = [mass_error m2_percent_error];
	%ctr_err = [ctr_err lc2_percent_error];
	%inertia_err = [inertia_err I2_percent_error];
    %condition_vector = [condition_vector cond(Y,2)];%sqrt(trace(transpose(Y)*Y)*trace(inv(transpose(Y)*Y)))];
i = i+1;
end

[~,condition_min] = find(condition_vector==min(min(condition_vector)))
[~,mass_error_min] = find(mass_error==min(min(mass_error)))
[~,ctr_err_min] = find(ctr_err==min(min(ctr_err)))
[~,inertia_err_min] = find(inertia_err==min(min(inertia_err)))
