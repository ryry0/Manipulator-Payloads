%This function generates a circular trajectory

delta_t = .01;
t_f = 2
time = 0:delta_t:t_f;
A = .05;

r = A + -.674;
l1 = .375;
l2 = .3;
filename = 'circular.txt';

x = A*sin(time*pi);
y = -A*cos(time*pi) + r;

x_vel = DiscreteDifferentiate(x, delta_t);
x_acc = DiscreteDifferentiate(x_vel, delta_t);

y_vel = DiscreteDifferentiate(y, delta_t);
y_acc = DiscreteDifferentiate(y_vel, delta_t);


x_state = [transpose(x), transpose(x_vel), transpose(x_acc), transpose(time)];
y_state = [transpose(y), transpose(y_vel), transpose(y_acc), transpose(time)];

[   q1_trajectory, q1_velocity, q1_acceleration, ...
    q2_trajectory, q2_velocity, q2_acceleration] = ... 
    GenerateArbitraryTrajectory(x_state, y_state, delta_t, l1, l2)


Q_traj = [transpose(q1_trajectory), ...
          transpose(q1_velocity), ...
          transpose(q1_acceleration), ...
          transpose(q2_trajectory), ...
          transpose(q2_velocity), ...
          transpose(q2_acceleration)];
dlmwrite(filename, Q_traj, 'precision', '%.4f')

AnimateTrajectory(q1_trajectory, q2_trajectory, l1, l2);
