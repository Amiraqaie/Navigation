clear;
clc;
% define earth angular velocity
wei = 2 * pi / (24 * 3600);
Wei_e = [0;
        0;
        wei];

%% define lattitude and longitude
lat = deg2rad(35);
lon = deg2rad(55);
h = 0;

%% define yaw and phi position
si_dot_values = [10, 20, 30, -10, -20, -30]; % deg/sec
phi_dot_values = [10, 20, 30, -10, -20, -30]; % deg/sec
num_samples_in_each_sidot_phidot_pair = 1000; % number of samples that will be record in each si dot and phi dot pairs
Ts = 0.001; % sample time of getting gyro data and table data

measured_gyro_outputs = zeros(length(si_dot_values) * length(phi_dot_values) * num_samples_in_each_sidot_phidot_pair, 3);
true_gyro_outputs = zeros(length(si_dot_values) * length(phi_dot_values) * num_samples_in_each_sidot_phidot_pair, 3);

index = 1;

for i = 1:length(si_dot_values)
    for j = 1:length(phi_dot_values)
        %% define si_dot and phi_dot and their initail values
        si = 30;
        phi = 30;
        si_dot = si_dot_values(i);
        phi_dot = phi_dot_values(i);
        
        for m = 1:num_samples_in_each_sidot_phidot_pair
            %% calculate the w_pt_b
            w_pt_b = [phi_dot * cos(si);
                      -phi_dot * sin(si);
                      si_dot];

            %% calculating DCM of T to P coordinate
            T_pt = [cos(si)        sin(si)     0;
                    -sin(si)       cos(si)     0;
                    0              0           1] * [1       0                   0;
                                                    0       cos(phi)        sin(phi);
                                                    0       -sin(phi)       cos(phi)];

            %% calculating DCM of N to E coordinate
            T_ne = [-sin(lat) * cos(lon),    -sin(lat) * sin(lon),   cos(lat);
                    -sin(lon),              cos(lon),               0;      
                    -cos(lat) * cos(lon),   -cos(lat) * sin(lon),   -sin(lat)];

            %% calculate the true gyro output
            w_bi_b = w_pt_b + T_pt * T_ne * Wei_e;

            %% simulating uncalibrated gyro output
            M = [(1 - 2e-5),        -3e-5,       6e-5;
                 -3e-5,          (1 + 6e-5),     -8e-5;
                 6e-5,             -8e-5,    (1 - 3e-5)];
            
            b_a_b = [5e-9;
                     -8e-9;
                     -6e-9];
        
        
            n_a_b = randn(3, 1) .* [2e-9;
                                    1e-9;
                                    2e-9];
            uncalibrated_gyro_output = M * w_bi_b + b_a_b + n_a_b;

            %% save the uncalibrated and true values of gyro
            measured_gyro_outputs(index, :) = transpose(uncalibrated_gyro_output);
            true_gyro_outputs(index, :) = transpose(w_bi_b);

            %% updating si and phi
            si = si + si_dot * Ts;
            phi = phi + phi_dot * Ts;
            index = index + 1;
        end
    end
end


%% solve the calibration problem for three channels of gyro
H = [ones(size(true_gyro_outputs, 1), 1), true_gyro_outputs];
pesudu_inverse = inv(transpose(H) * H) * transpose(H);

% bias vector and M matrix
calibration_matrix = pesudu_inverse * measured_gyro_outputs;
bias_calibration = calibration_matrix(1, :);
M_calibration = calibration_matrix(2:4, :);

% Compute percentage error for bias
bias_error_percent = 100 * (bias_calibration - b_a_b') ./ b_a_b';

% Compute percentage error for each element of M
M_error_percent = 100 * (M_calibration - M) ./ M;

% Display results
disp('Gyro Bias estimation error in percentage:');
disp(array2table(bias_error_percent, 'VariableNames', {'X', 'Y', 'Z'}));

disp('Gyro M matrix estimation error in percentage:');
disp(array2table(M_error_percent, ...
    'VariableNames', {'X', 'Y', 'Z'}, ...
    'RowNames', {'X_row', 'Y_row', 'Z_row'}));