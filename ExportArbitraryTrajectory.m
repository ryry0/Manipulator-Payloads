%%
% This function can generate and export to a file, 
% an arbitrary path when supplied with a 
% list of states for the x and y axis. The structure of the list must look
% like this:
% n_state = [n_pos, n_vel, n_acc, time;
%            n_pos2, n_vel2, n_acc2, time2;]
% and so on. The x position and y position do not necessarily require the
% same amount of data points, but the must end at the same time or weird
% things will happen. 
% Trajectory specifications

function ExportArbitraryTrajectory(x_state, y_state, delta_t, l1, l2, filename)
[   q1_trajectory, q1_velocity, q1_acceleration, ...
    q2_trajectory, q2_velocity, q2_acceleration] = ...
    GenerateArbitraryTrajectory(x_state, y_state, delta_t, l1, l2);

Q_traj = [transpose(q1_trajectory), ...
          transpose(q1_velocity), ...
          transpose(q1_acceleration), ...
          transpose(q2_trajectory), ...
          transpose(q2_velocity), ...
          transpose(q2_acceleration)];
dlmwrite(filename, Q_traj, 'precision', '%.4f')
end
