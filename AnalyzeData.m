%%
% Physical parameters of the Pelican robot arm. (m, kg, s)

% Length of links
l1 = 0.375;
l2 = 0.3;

% Distance from origin of each link to center of mass
lc1 = 0.195;
lc2_base = 0.22;

% Mass of links
m1 = 2.883;
m2_base = 1.085;

% Inertia relative to center of mass
I1 = 0.0345;
I2_base = 0.013;

% Gravity term
g = 9.81;

%load parameters
ml = 2.883;
radius_l = 0.0254;
Il = (ml * (radius_l^2))/2;
lcl = 0.2744;

%loaded
if (loaded == 1)
    I2 = 0.0296;
    lc2 = 0.2744;
    m2 = 3.401;
elseif (loaded == 2)
    I2 = 0.0435;
    lc2 = 0.2849;
    m2 = 5.6209;
else
    I2 = 0.013;
    lc2 = 0.22;
    m2 = 1.085;
end

t_0 = 0;
t_final = 2;
delta_t = 0.001;
time = t_0 : delta_t : t_final;
%num_samples = 2000; % this is the number of samples across the whole
%trajectory

%cutoff frequency for low pass filter
fc = .05;
%scaling_factor = 1.5;

%if (filter == 1)
    torque_1 = LowPass(data(:,14),fc);
    torque_1 = torque_1 .* scaling_factor;

    torque_2 = LowPass(data(:,15),fc);
    torque_2 = torque_2 .* scaling_factor;

    q1_trajectory	= LowPass(data(:, 8),fc);
    q1_velocity		= LowPass(data(:, 9),fc);

    q1_acceleration = DiscreteDifferentiate(data(:,9),delta_t);
    q1_acceleration = LowPass(q1_acceleration,fc);

    q2_trajectory	= LowPass(data(:,11),fc);
    q2_velocity		= LowPass(data(:,12),fc);

    q2_acceleration = DiscreteDifferentiate(data(:,12),delta_t);
    q2_acceleration = LowPass(q2_acceleration, fc);

%else
%    torque_1 = smooth(data(:,14),21);
%    torque_1 = torque_1 .* scaling_factor;
%
%    torque_2 = smooth(data(:,15),21);
%    torque_2 = torque_2 .* scaling_factor;
%
%    q1_trajectory	= smooth(data(:, 8),21);
%    q1_velocity		= smooth(data(:, 9),21);
%
%    q1_acceleration = DiscreteDifferentiate(data(:,9), delta_t);
%    q1_acceleration = smooth(q1_acceleration,21);
%
%    q2_trajectory	= smooth(data(:,11),21);
%    q2_velocity		= smooth(data(:,12),21);
%
%    q2_acceleration = DiscreteDifferentiate(data(:,12), delta_t);
%    q2_acceleration = smooth(q2_acceleration, 21);
%end
%%
% 'true' parameters
chi1 = m1 * lc1^2 + m2 * (l1^2 + lc2^2) + I1 + I2;
chi2 = m2 * l1 * lc2;
chi3 = m2 * lc2^2 + I2;
chi4 = m1* lc1 + m2*l1;
chi5 = m2 * lc2;

chi = [chi1 ; chi2 ; chi3 ; chi4 ; chi5];

%%
Y = double.empty;
torque = double.empty;
%for all time. 
%in steps proportional to num_samples. so if num samples is 3, 1/3rd of
%time passes before the next sample is "calclulated"
%for n = 1:floor(length(time)/(num_samples - 1)):length(time)
for n = 1:length(time)
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

printmat_v2(chi_results, 'Chi Results', 'chi1 chi2 chi3 chi4 chi5', 'chi& chihat& chi_%_error', '&')
printmat_v2(param_results, 'Parameter Results', 'mass ctr_of_mass inertia', 'orig& calc& %_error', '&')
