%% This is meant to give me batch analysis of the data
clc

loaded = 1;
data_begin = 1;
num_files = 3;
data_files = cell(num_files,1);
% data_files{1} = 'Mani_Data\Loaded\data_large_vert_full\data_large_vert_full_exp1.txt';
% data_files{2} = 'Mani_Data\Loaded\data_large_vert_full\data_large_vert_full_exp2.txt';
% data_files{3} = 'Mani_Data\Loaded\data_large_vert_full\data_large_vert_full_exp3.txt';

% data_files{1} = 'Mani_data\Loaded\data_second_link_10lb_exp1\data_second_link_10lb_exp1.txt';
% data_files{2} = 'Mani_data\Loaded\data_second_link_10lb_exp1\data_second_link_10lb_exp2.txt';
% data_files{3} = 'Mani_data\Loaded\data_second_link_10lb_exp1\data_second_link_10lb_exp3.txt';

% data_files{1} = 'Mani_data\Loaded\data_oscillation_15deg_5lb_exp1\data_oscillation_15deg_5lb_exp1.txt';
% data_files{2} = 'Mani_data\Loaded\data_oscillation_15deg_5lb_exp1\data_oscillation_15deg_5lb_exp2.txt';
% data_files{3} = 'Mani_data\Loaded\data_oscillation_15deg_5lb_exp1\data_oscillation_15deg_5lb_exp3.txt';

% data_files{1} = 'Mani_data\Loaded\data_oscillation_15deg_10lb_exp1\data_oscillation_15deg_10lb_exp1.txt';
% data_files{2} = 'Mani_data\Loaded\data_oscillation_15deg_10lb_exp1\data_oscillation_15deg_10lb_exp2.txt';

data_files{1} = 'Mani_data\Loaded\data_oscillate_30deg_5lb_exp1\data_oscillate_30deg_5lb_exp1.txt';
data_files{2} = 'Mani_data\Loaded\data_oscillate_30deg_5lb_exp1\data_oscillate_30deg_5lb_exp2.txt';
data_files{3} = 'Mani_data\Loaded\data_oscillate_30deg_5lb_exp1\data_oscillate_30deg_5lb_exp3.txt';

% data_files{1} = 'Mani_data\Loaded\data_oscillate_30deg_10lb_exp1\data_oscillate_30deg_10lb_exp1.txt';
% data_files{2} = 'Mani_data\Loaded\data_oscillate_30deg_10lb_exp1\data_oscillate_30deg_10lb_exp2.txt';
% data_files{3} = 'Mani_data\Loaded\data_oscillate_30deg_10lb_exp1\data_oscillate_30deg_10lb_exp3.txt';

calc_mass1 = double.empty;
calc_ctr1 = double.empty;
calc_I1 = double.empty;

mass_err1 = double.empty;
ctr_err1 = double.empty;
I_err1 = double.empty;

calc_mass36 = double.empty;
calc_ctr36 = double.empty;
calc_I36 = double.empty;

mass_err36 = double.empty;
ctr_err36 = double.empty;
I_err36 = double.empty;

scaling_factor = 1
%%
%-----------------------------------------------
for dataset = 1:num_files
data = importdata(data_files{dataset}, ' ');
data = data(data_begin:end, :);
AnalyzeData

calc_mass1 = [calc_mass1; m2_hat];
calc_ctr1  = [calc_ctr1; lc2_hat];
calc_I1    = [calc_I1; I2_hat];

mass_err1 = [mass_err1; m2_percent_error];
ctr_err1  = [ctr_err1; lc2_percent_error];
I_err1    = [I_err1; I2_percent_error];

 end
%-----------------------------------------------
scaling_factor = 1.36
for dataset = 1:num_files
data = importdata(data_files{dataset}, ' ');
data = data(data_begin:end, :);
AnalyzeData

calc_mass36 = [calc_mass36; m2_hat];
calc_ctr36  = [calc_ctr36; lc2_hat];
calc_I36    = [calc_I36; I2_hat];

mass_err36 = [mass_err36; m2_percent_error];
ctr_err36  = [ctr_err36; lc2_percent_error];
I_err36    = [I_err36; I2_percent_error];
end
%-----------------------------------------------
calc_mass1
calc_ctr1
calc_I1

mass_err1 
ctr_err1 
I_err1 

calc_mass36 
calc_ctr36 
calc_I36 

mass_err36 
ctr_err36 
I_err36 
