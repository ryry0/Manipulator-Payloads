%this script will be used to optimize the Simulation in different parameters


mass_error = double.empty;
ctr_err = double.empty;
inertia_err = double.empty;
condition_vector = double.empty;
%for t_final = 1:.001:5
final_t = 6;
%q_final = .001:.001:pi/4;
q_final = pi/6;
%sin_time = 0:.001:6;
end_time = 1:.01:final_t;

freq = .25:.01:3;
%data = datasecondlinkloaded5lbexp1(1:2000,:);

for i = 1:length(end_time)
	t_final = end_time(i);
	q_traj = [0,0,0,0; 0, 0, 0, end_time(i)];

	 q2_traj = [0, 0, 0, 0;
	 q_final, 0, 0, end_time(i)/5;
	 -q_final, 0, 0, 2*end_time(i)/5;
	 q_final, 0, 0, 3*end_time(i)/5;
	 -q_final, 0, 0, 4*end_time(i)/5;
	 0, 0, 0, end_time(i)];
 
 [   q1_trajectory, q1_velocity, q1_acceleration, ...
     q2_trajectory, q2_velocity, q2_acceleration] = ...
     GenArbJointTraj(q_traj, q2_traj, delta_t, l1, l2);

    %q2_trajectory = sin(2*pi*freq(i)*sin_time); 
    %q2_velocity = DiscreteDifferentiate(q2_trajectory, .001);
    %q2_acceleration = DiscreteDifferentiate(q2_velocity, .001);

%q2_trajectory = LowPass(data(:,11), .05);
%q2_acceleration = LowPass(DiscreteDifferentiate(data(:,12), .001), .05);
Y_2 = sin(q2_trajectory);
Y_1 = q2_acceleration;
%
Y = [transpose(Y_1), transpose(Y_2)];
%torque = LowPass(data(:,15), .05) *1.36;
torque = .0435*q2_acceleration + 5.6209*9.8*.2849*.5*sin(q2_trajectory);
torque = transpose(torque);
%
a = (transpose(Y)*Y)\transpose(Y) * torque
%condition_vector(i) = cond(Y,2);
%end
%for i = 1:length(sin_time)

%%
% Generate Joint Space Trajectory
%q_final(i)

    %q1_trajectory = sin(2*pi*freq*sin_time); 
    %q1_velocity = DiscreteDifferentiate(q1_trajectory, .001);
    %q1_acceleration = DiscreteDifferentiate(q1_velocity, .001);
    %q1_trajectory = zeros(1,length(sin_time));
    %q1_velocity = zeros(1,length(sin_time));
    %q1_acceleration = zeros(1,length(sin_time));
    %Sim_Arb_Client
        
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
