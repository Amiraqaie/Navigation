clear;
clc;

%% do accell calibration process
gyro_calibration

%% convert accell output data and correct them using calibration matrix
calibrated_gyro_output = (inv(M_calibration) * (measured_gyro_outputs - bias_calibration)')';

%% compare calibrated output to true value of accell
error_percentage = 100 * abs(calibrated_gyro_output - true_gyro_outputs) ./ true_gyro_outputs;
num_blocks = size(error_percentage, 1) / num_samples_in_each_sidot_phidot_pair;
error_percentage_downsampled = zeros(num_blocks, 3);
% Loop through each block and compute the mean
for i = 1:num_blocks
    idx_start = (i-1)*num_samples_in_each_sidot_phidot_pair + 1;
    idx_end = i*num_samples_in_each_sidot_phidot_pair;
    block = error_percentage(idx_start:idx_end, :);
    error_percentage_downsampled(i, :) = mean(block, 1);  % mean across rows
end

%% Create the table with each combination of si and phi
si_dot_len = length(si_dot_values);
phi_dot_len = length(phi_dot_values);
num_rows = si_dot_len * phi_dot_len;

% Initialize arrays for si and phi values
si_dot_array = repmat(si_dot_values, phi_dot_len, 1);
phi_dot_array = repmat(phi_dot_values', si_dot_len, 1);

% Convert the arrays to column vectors for the table
si_dot_array = si_dot_array(:);
phi_dot_array = phi_dot_array(:);

% Create the table with units in the column names
T = table(si_dot_array, phi_dot_array, error_percentage_downsampled(:, 1), error_percentage_downsampled(:, 2), error_percentage_downsampled(:, 3), ...
          'VariableNames', {'si_dot_deg/sec', 'phi_dot_deg/sec', 'gx_error_percent', 'gy_error_percent', 'gz_error_percent'});

% Display the table
disp(T);