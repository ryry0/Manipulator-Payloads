%%This will be a simulation of the 2 DOF manipulator
% The main variables are left undefined so that you can 
% customize them at runtime.


%%
% Physical parameters of the Pelican robot arm. (m, kg, s)

% Length of links
%l1 = 0.26;
%l2 = 0.26;

% Distance from origin of each link to center of mass
%lc1 = 0.0983;
%lc2_base = 0.0229;

% Mass of links
%m1 = 6.5225;
%m2_base = 2.0458;

% Inertia relative to center of mass
%I1 = 0.1213;
%I2_base = 0.0116;

% Gravity term
%g = 9.81;

[m2, lc2, I2] = AddLoad(m2_base, lc2_base, I2_base, 2, l2 - .0254, .0254^2);

%%
% Trajectory specifications

x_initial = 0; 
x_final = 0;
x_velocity_initial = 0;
x_velocity_final = 0;
x_acceleration_initial = 0;
x_acceleration_final = 0;

y_initial = -.51;
y_final = -.3;
y_velocity_initial = 0;
y_velocity_final = 0;
y_acceleration_initial = 0;
y_acceleration_final = 0;

t_0 = 0;
t_final = 2;
delta_t = 0.001;
time = t_0 : delta_t : 2;
num_samples = 2000 % this is the number of samples across the whole
%trajectory

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
%the following variables hold the x and y coords of position 
x_q1 = l1 * sin(q1_trajectory); 
y_q1 = -l1 * cos(q1_trajectory);

%the following are the coordinates of the termination link
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

Y = double.empty;
torque = double.empty;
for n = 1:floor(length(time)/(num_samples - 1)):length(time)
    q1_accel_n = q1_acceleration(n);
    q1_veloc_n = q1_velocity(n);
    q1_traj_n = q1_trajectory(n);
    q2_accel_n = q2_acceleration(n);
    q2_veloc_n = q2_velocity(n);
    q2_traj_n = q2_trajectory(n);
    Y_12n = cos(q2_traj_n)*(2*q1_accel_n + q2_accel_n) - sin(q2_traj_n)*(q1_veloc_n^2 + 2*q1_veloc_n*q2_veloc_n);
    Y_22n = cos(q2_traj_n)*q1_accel_n + sin(q2_traj_n)*(q1_veloc_n^2);
    Y_n = [q1_accel_n,  Y_12n,  q2_accel_n,              g*sin(q1_traj_n),  g*sin(q1_traj_n + q2_traj_n); ...
           0,           Y_22n,  q1_accel_n + q2_accel_n, 0,                 g*sin(q1_traj_n + q2_traj_n)];
    torque_n = [torque_1(n); torque_2(n)];
    torque = [torque; torque_n];
       Y = [Y; Y_n];
end

chihat = (transpose(Y) * Y)\ transpose(Y) * torque;
chi_percent_error = abs(chihat - chi)./chi * 100;

m2_hat = (chihat(4)-m1*lc1)/l1;
lc2_hat = chihat(5)/m2_hat;
I2_hat = chihat(3)-m2_hat*lc2_hat^2;

m2_percent_error = abs(m2_hat - m2)./m2 * 100;
lc2_percent_error = abs(lc2_hat - lc2)./lc2 * 100;
I2_percent_error = abs(I2_hat - I2)./I2 * 100;

chi_results = [chi chihat chi_percent_error];
param_results = ...
    [m2 m2_hat m2_percent_error; ...
    lc2 lc2_hat lc2_percent_error; ...
    I2 I2_hat I2_percent_error];

printmat(chi_results, 'Chi Results', 'chi1 chi2 chi3 chi4 chi5', 'chi chihat chi_%_error')
printmat(param_results, 'Parameter Results', 'mass ctr_of_mass inertia', 'orig calc %_error')
