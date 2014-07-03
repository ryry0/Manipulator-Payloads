function [ x_trajectory ] =...%, x_velocity, x_acceleration] = ...
        GenerateTaskTrajectory(time, delta_t, coefficients)
    
% Generates a 5th order trajectory in task space
%
x_trajectory = coefficients(1) + coefficients(2)*time ...
        + coefficients(3)*time.^2 + coefficients(4)*time.^3 ...
        + coefficients(5)*time.^4 + coefficients(6)*time.^5;
    

%x_velocity = DiscreteDifferentiate(x_trajectory, delta_t);
%x_acceleration = DiscreteDifferentiate(x_velocity, delta_t);            
end

