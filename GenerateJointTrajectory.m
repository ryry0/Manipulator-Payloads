function [q1, q1_velocity, q1_acceleration, q2, q2_velocity, q2_acceleration] = ...
    GenerateJointTrajectory(x, y, delta_t, l_1, l_2)
% Calculates the joint positions, velocities, and accelerations 
% using inverse kinematics
%   Detailed explanation goes here

q2 = acos( (x.^2 + y.^2 - l_1^2 - l_2^2) ./ (2 * l_1 * l_2) );
q1 = atan2(x,-y) - atan( ( l_2 .* sin(q2) ) ./ (l_1 + l_2 .* cos(q2) ) );

q1_velocity = DiscreteDifferentiate(q1, delta_t);
q1_acceleration = DiscreteDifferentiate(q1_velocity, delta_t);

q2_velocity = DiscreteDifferentiate(q2, delta_t);
q2_acceleration = DiscreteDifferentiate(q2_velocity, delta_t);
end

