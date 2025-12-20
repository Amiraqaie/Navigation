clear;
clc;

%% do accel calibration process
accel_calibration

%% convert accel output data and correct them using calibration matrix
calibrated_accel_output = (inv(M_calibration) * (measured_accel_outputs - bias_calibration)')';

%% compare calibrated output to true value of accel
error_percentage = 100 * abs(calibrated_accel_output - true_accel_outputs) ./ true_accel_outputs;

%% Create the table with each combination of si and phi
si_len = length(si_positions);
phi_len = length(phi_positions);
num_rows = si_len * phi_len;

% Initialize arrays for si and phi values
si_array = repmat(si_positions, phi_len, 1);
phi_array = repmat(phi_positions', si_len, 1);

% Convert the arrays to column vectors for the table
si_array = si_array(:);
phi_array = phi_array(:);

% Create the table with units in the column names
T = table(si_array, phi_array, error_percentage(:, 1), error_percentage(:, 2), error_percentage(:, 3), ...
          'VariableNames', {'si_angle_deg', 'phi_angle_deg', 'ax_error_percent', 'ay_error_percent', 'az_error_percent'});

% Display the table
disp(T);