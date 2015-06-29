%This function is capable of animating a given joint trajectory.
%it requires the trajectory.
function AnimateTrajectory(q1_trajectory, q2_trajectory, l1, l2)
%the following variables hold the x and y coords of position 
axis_size = .7;
x_q1 = l1 * sin(q1_trajectory); 
y_q1 = -l1 * cos(q1_trajectory);

%the following are the coordinates of the termination link
x_tot = x_q1 + l2 .* sin(q1_trajectory + q2_trajectory);
y_tot = y_q1 - l2 .* cos(q1_trajectory + q2_trajectory);
[~, num_cols] = size(x_tot);

for n = 1:num_cols
plot([0,x_q1(n)], [0, y_q1(n)], [x_q1(n),x_tot(n)], [y_q1(n), y_tot(n)]);
axis equal;
axis([-axis_size, axis_size, -axis_size, axis_size]);
M(n)=getframe;
end

num_times=3;
fps=10;
movie(M,num_times, fps);

end
