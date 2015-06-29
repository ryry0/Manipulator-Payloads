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
end_time = final_t:.01:final_t;

freq = .25:.01:3;
data = importdata('Mani_data\Loaded\data_second_link_loaded\data_second_link_loaded5lb_exp1.txt', ' ');
AnalyzeData
A = Y;

	t_final = end_time(i);
	q_traj = [0,0,0,0; 0, 0, 0, end_time(i)];

	 q2_traj = [0, 0, 0, 0;
% 	 q_final, 0, 0, end_time(i)/5;
% 	 -q_final, 0, 0, 2*end_time(i)/5;
% 	 q_final, 0, 0, 3*end_time(i)/5;
% 	 -q_final, 0, 0, 4*end_time(i)/5;
	 q_final, 0, 0, end_time(i)];
 
 [   q1_trajectory, q1_velocity, q1_acceleration, ...
     q2_trajectory, q2_velocity, q2_acceleration] = ...
     GenArbJointTraj(q_traj, q2_traj, delta_t, l1, l2);
     Sim_Arb_Client
     B = Y;
     
    %q2_trajectory = sin(2*pi*freq(i)*sin_time); 
    %q2_velocity = DiscreteDifferentiate(q2_trajectory, .001);
    %q2_acceleration = DiscreteDifferentiate(q2_velocity, .001);

%q2_trajectory = LowPass(data(:,11), .05);
%q2_acceleration = LowPass(DiscreteDifferentiate(data(:,12), .001), .05);
% Y_2 = sin(q2_trajectory);
% Y_1 = q2_acceleration;
% %
% Y = [transpose(Y_1), transpose(Y_2)];
% %torque = LowPass(data(:,15), .05) *1.36;
% torque = .0435*q2_acceleration + 5.6209*9.8*.2849*.5*sin(q2_trajectory);
% torque = transpose(torque);
% %
% a = (transpose(Y)*Y)\transpose(Y) * torque
%condition_vector(i) = cond(Y,2);


    
        
 

