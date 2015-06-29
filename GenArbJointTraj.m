%%
% This function can generate an arbitrary path when supplied with a 
% list of states for the x and y axis. The structure of the list must look
% like this:
% n_state = [n_pos, n_vel, n_acc, time;
%            n_pos2, n_vel2, n_acc2, time2;]
% and so on. The x position and y position do not necessarily require the
% same amount of data points, but the must end at the same time or weird
% things will happen. x_state represents the first link, y_state represents
% the second link
% Trajectory specifications

function [   q1_trajectory, q1_velocity, q1_acceleration, ...
    q2_trajectory, q2_velocity, q2_acceleration] = ... 
    GenerateArbitraryTrajectory(x_state, y_state, delta_t, l1, l2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%time = t_0 : delta_t : t_final;
[x_num_rows, ~] = size(x_state);
[y_num_rows, ~] = size(y_state);
STATE_POS_COL = 1;
STATE_VEL_COL = 2;
STATE_ACC_COL = 3;
TIME_COL = 4;

%%
% Generate the polynomial coefficients of the trajectory
%for the number of rows, generate n-1 Trajectory coefficients.
x_coefficient_matrix = double.empty;
y_coefficient_matrix = double.empty;

for n=1:x_num_rows-1
x_coefficients = GenerateTaskTrajectoryCoeff(   x_state(n, TIME_COL), ...
                                                x_state(n+1, TIME_COL), ...
                                                x_state(n, STATE_POS_COL), ...
                                                x_state(n, STATE_VEL_COL), ...
                                                x_state(n, STATE_ACC_COL), ...
                                                x_state(n+1, STATE_POS_COL), ...
                                                x_state(n+1, STATE_VEL_COL), ...
                                                x_state(n+1, STATE_ACC_COL));
x_coefficient_matrix = [x_coefficient_matrix x_coefficients];
end

for n=1:y_num_rows-1
y_coefficients = GenerateTaskTrajectoryCoeff(   y_state(n, TIME_COL), ...
                                                y_state(n+1, TIME_COL), ...
                                                y_state(n, STATE_POS_COL), ...
                                                y_state(n, STATE_VEL_COL), ...
                                                y_state(n, STATE_ACC_COL), ...
                                                y_state(n+1, STATE_POS_COL), ...
                                                y_state(n+1, STATE_VEL_COL), ...
                                                y_state(n+1, STATE_ACC_COL));

y_coefficient_matrix = [y_coefficient_matrix y_coefficients];
end
%%
% Generate Task Space Trajectories
x_traj_complete = double.empty;
y_traj_complete = double.empty;

for n=1:x_num_rows-1
%create a time range from the first time to the final time minus a delta t
if (n == x_num_rows-1) %last iteration
    time=x_state(n,TIME_COL):delta_t:x_state(n+1, TIME_COL);
else
    time=x_state(n,TIME_COL):delta_t:x_state(n+1, TIME_COL)-delta_t;
end
x_trajectory = GenerateTaskTrajectory(time, delta_t, x_coefficient_matrix(:,n));
x_traj_complete = [x_traj_complete x_trajectory];
end

for n=1:y_num_rows-1
%create a time range from the first time to the final time minus a delta t
if (n == y_num_rows-1)
    time=y_state(n,TIME_COL):delta_t:y_state(n+1, TIME_COL);
else
    time=y_state(n,TIME_COL):delta_t:y_state(n+1, TIME_COL)-delta_t;
end
y_trajectory = GenerateTaskTrajectory(time, delta_t, y_coefficient_matrix(:,n));
y_traj_complete = [y_traj_complete y_trajectory];
end

q1_trajectory = x_traj_complete;
q2_trajectory = y_traj_complete;


q1_velocity = DiscreteDifferentiate(q1_trajectory, delta_t);
q1_acceleration = DiscreteDifferentiate(q1_velocity, delta_t);

q2_velocity = DiscreteDifferentiate(q2_trajectory, delta_t);
q2_acceleration = DiscreteDifferentiate(q2_velocity, delta_t);

Q_traj = [transpose(q1_trajectory), ...
          transpose(q1_velocity), ...
          transpose(q1_acceleration), ...
          transpose(q2_trajectory), ...
          transpose(q2_velocity), ...
          transpose(q2_acceleration)];
dlmwrite('oscillate.txt', Q_traj, 'precision', '%.4f')
end
