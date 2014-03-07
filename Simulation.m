%%This will be a simulation of the 2 DOF manipulator
%


%%
% Physical parameters of the Pelican robot arm. (m, kg, s)

% Length of links
l1 = 0.26;
l2 = 0.26;

% Distance from origin of each link to center of mass
lc1 = 0.0983;
lc2 = 0.0229;

% Mass of links
m1 = 6.5225;
m2 = 2.0458;

% Inertia relative to center of mass
I1 = 0.1213;
I2 = 0.0116;

% Gravity term
g = 9.81;


%%
% Trajectory specifications

x_initial = 0; 
x_final = .3;
x_velocity_initial = 0;
x_velocity_final = 0;
x_acceleration_initial = 0;
x_acceleration_final = 0;

y_initial = -.5;
y_final = .4;
y_velocity_initial = 0;
y_velocity_final = 0;
y_acceleration_initial = 0;
y_acceleration_final = 0;

t_0 = 0;
t_final = 2;
delta_t = 0.1;
time = t_0 : delta_t : 2;

%%
% Generate the polynomial coefficients of the trajectory
x_coefficients = GenerateTaskTrajectoryCoeff(   t_0, ...
                                                t_final, ...
                                                x_initial, ...
                                                x_velocity_initial, ...
                                                x_acceleration_initial, ...
                                                x_final, ...
                                                x_velocity_final, ...
                                                x_acceleration_final);

y_coefficients = GenerateTaskTrajectoryCoeff(   t_0, ...
                                                t_final, ...
                                                y_initial, ...
                                                y_velocity_initial, ...
                                                y_acceleration_initial, ...
                                                y_final, ...
                                                y_velocity_final, ...
                                                y_acceleration_final);
                                                
%%
% Generate Task Space Trajectories

x_trajectory = GenerateTaskTrajectory(time, delta_t, x_coefficients);
y_trajectory = GenerateTaskTrajectory(time, delta_t, y_coefficients);

%%
% Generate Joint Space Trajectory

[   q1_trajectory, q1_velocity, q1_acceleration, ...
    q2_trajectory, q2_velocity, q2_acceleration] = ...
    GenerateJointTrajectory(x_trajectory, y_trajectory, delta_t, l1, l2);

%plot(time, x_trajectory)
%plot(time, q1_trajectory)
x_q1 = l1 * sin(q1_trajectory);
y_q1 = -l1 * cos(q1_trajectory);

x_tot = x_q1 + l2 .* sin(q1_trajectory + q2_trajectory);
y_tot = y_q1 - l2 .* cos(q1_trajectory + q2_trajectory);

%{
for n = 1:length(time)
plot([0,x_q1(n)], [0, y_q1(n)], [x_q1(n),x_tot(n)], [y_q1(n), y_tot(n)])
axis([-.6 .6 -.6 .6]) 
end
%}
torque_1 = (m1 * lc1^2 + m2 * l1^2 + m2 * lc2^2 ...
            + 2 * m2 * lc2 * cos(q2_trajectory) + I1 + I2) .* q1_acceleration ...
            + (m2 * lc2^2 + m2 * l1 * lc2 * cos(q2_trajectory) + I2) .* q2_acceleration ...
            - 2 * m2 * l1 * lc2 * sin(q2_trajectory) .* q1_velocity .* q2_velocity ...
            - m2 * l1 * lc2 * sin(q2_trajectory).* q1_velocity .^2 ...
            + (m1 * lc1 + m2 * l1) * g * sin(q1_trajectory) ...
            + m2 * g * lc2 * sin(q1_trajectory + q2_trajectory);
        
torque_2 = (m2 * lc2^2 + m2 * l1 * lc2 * cos(q2_trajectory) + I2)  .* q1_acceleration ...
            + (m2 * lc2^2 + I2) * q2_acceleration ...
            + m2 * l1 * lc2 * sin(q2_trajectory) .* q1_velocity .^ 2 ...
            + m2 * g * lc2 * sin(q1_trajectory + q2_trajectory);
        
chi1 = m1 * lc1^2 + m2 * (l1^2 + lc2^2) + I1 + I2;
chi2 = m2 * l1 * lc2;
chi3 = m2 * lc2^2 + I2;
chi4 = m1* lc1 + m2*l1;
chi5 = m2 * lc2;

chi = [chi1 ; chi2 ; chi3 ; chi4 ; chi5];







                                                
