%% This is meant to give me batch analysis of the data
clc

loaded = 2;
data_begin = 1;
num_files = 3;
data_files = cell(num_files,1);

% data_files{1} = 'Mani_data\Loaded\data_second_link_loaded\data_second_link_loaded5lb_exp1.txt';
% data_files{2} = 'Mani_data\Loaded\data_second_link_loaded\data_second_link_loaded5lb_exp2.txt';
% data_files{3} = 'Mani_data\Loaded\data_second_link_loaded\data_second_link_loaded5lb_exp3.txt';

data_files{1} = 'Mani_data\Loaded\data_second_link_10lb_exp1\data_second_link_10lb_exp1.txt';
data_files{2} = 'Mani_data\Loaded\data_second_link_10lb_exp1\data_second_link_10lb_exp2.txt';
data_files{3} = 'Mani_data\Loaded\data_second_link_10lb_exp1\data_second_link_10lb_exp3.txt';

% data_files{1} = 'Mani_data\Loaded\data_oscillation_15deg_5lb_exp1\data_oscillation_15deg_5lb_exp1.txt';
% data_files{2} = 'Mani_data\Loaded\data_oscillation_15deg_5lb_exp1\data_oscillation_15deg_5lb_exp2.txt';
% data_files{3} = 'Mani_data\Loaded\data_oscillation_15deg_5lb_exp1\data_oscillation_15deg_5lb_exp3.txt';
%  

%data_files{1} = 'Mani_data\Loaded\data_oscillation_15deg_10lb_exp1\data_oscillation_15deg_10lb_exp1.txt';
%data_files{2} = 'Mani_data\Loaded\data_oscillation_15deg_10lb_exp1\data_oscillation_15deg_10lb_exp2.txt';

% data_files{1} = 'Mani_data\Loaded\data_oscillate_30deg_5lb_exp1\data_oscillate_30deg_5lb_exp1.txt';
% data_files{2} = 'Mani_data\Loaded\data_oscillate_30deg_5lb_exp1\data_oscillate_30deg_5lb_exp2.txt';
% data_files{3} = 'Mani_data\Loaded\data_oscillate_30deg_5lb_exp1\data_oscillate_30deg_5lb_exp3.txt';

% data_files{1} = 'Mani_data\Loaded\data_oscillate_30deg_10lb_exp1\data_oscillate_30deg_10lb_exp1.txt';
% data_files{2} = 'Mani_data\Loaded\data_oscillate_30deg_10lb_exp1\data_oscillate_30deg_10lb_exp2.txt';
% data_files{3} = 'Mani_data\Loaded\data_oscillate_30deg_10lb_exp1\data_oscillate_30deg_10lb_exp3.txt';

calc_I1 = double.empty;
I_err1 = double.empty;

calc_I36 = double.empty;
I_err36 = double.empty;

scaling_factor = 1
%%
%-----------------------------------------------
for dataset = 1:num_files
data = importdata(data_files{dataset}, ' ');
data = data(data_begin:end, :);
AnalyzeData
%%
A = [Y(:,2) Y(:,3) Y(:,5)];

chihat = (transpose(A)*A)\transpose(A)*torque;

I2_hat = chihat(2)-m2*lc2^2;
I2_percent_error = abs(I2_hat - I2)./I2 * 100;
%%
calc_I1    = [calc_I1; I2_hat];

I_err1    = [I_err1; I2_percent_error];

 end
%-----------------------------------------------
scaling_factor = 1.36
for dataset = 1:num_files
data = importdata(data_files{dataset}, ' ');
data = data(data_begin:end, :);
AnalyzeData
%%
A = [Y(:,2) Y(:,3) Y(:,5)];

chihat = (transpose(A)*A)\transpose(A)*torque;

I2_hat = chihat(2)-m2*lc2^2;
I2_percent_error = abs(I2_hat - I2)./I2 * 100;
%%
calc_I36    = [calc_I36; I2_hat];

I_err36    = [I_err36; I2_percent_error];
end
%-----------------------------------------------
%%
%-----------------------------------------------
calc_I1

I_err1 

calc_I36 

I_err36 
