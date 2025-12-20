clear;
clc;

%% define lattitude and longitude
lat = deg2rad(35);
lon = deg2rad(55);
h = 1000;

%% define yaw and phi position
si_positions = [10, 110, 250, 270, 30, 150];
phi_positions = [-60, -30, 30, 60];
num_samples_in_each_si_phi_pair = 1000000;
recorded_accel_ouptputs = zeros(num_samples_in_each_si_phi_pair, 3);
measured_accel_outputs = zeros(length(si_positions) * length(phi_positions), 3);
true_accel_outputs = zeros(length(si_positions) * length(phi_positions), 3);

index = 1;

for i = 1:length(si_positions)
    for j = 1:length(phi_positions)
        %% define si and phi position
        si = si_positions(i);
        phi = phi_positions(j);

        %% calculating the [g]n
        R0 = 6378137;
        Rp = 6356752.31425;
        wei = 2 * pi / (24 * 3600);
        e = 0.0818191908425;
        f = 1 / (298.257223563);
        mu = 3.986004418 * 1e14;
        Rn = R0 / sqrt( 1 - e^2 * sin(lat)^2);
        Rm = R0 * ( 1 - e^2) / ( 1 - e^2 * sin(lat)^2)^(3/2);
        g0 = 9.7803253359 * (1 + 0.001931853 * sin(lat)^2) / sqrt( 1 - e^2 * sin(lat)^2);
        q = 1 + (wei^2 * R0^2 * Rp / mu) + (1 - 2 * sin(lat)^2) * f;
        p = 1 - (2 * q * h / R0) + (3 * h^2 / R0^2);
        g = [-8.08 * 1e-9 * h * sin(2 * lat);
            0;
            p * g0];
        
        %% calculating ideal accel output
        T_bt = [cos(si)        sin(si)     0;
                -sin(si)       cos(si)     0;
                0              0           1] * [1       0                   0;
                                                0       cos(phi)        sin(phi);
                                                0       -sin(phi)       cos(phi)];
        f_bi_b = - T_bt * g;

        %% simulating uncalibrated accel output
        M = [(1 + 7e-5),        -5e-5,       7e-5;
             -5e-5,          (1 - 9e-5),     -8e-5;
             7e-5,             -8e-5,    (1 + 8e-5)];
        
        b_a_b = [-9e-3;
                 -8e-3;
                 -7e-3];
        
        for m = 1:num_samples_in_each_si_phi_pair
            n_a_b = randn(3, 1) .* [1e-3;
                                   2e-3;
                                   2e-3];
            recorded_accel_ouptputs(m, :) = transpose(M * f_bi_b + b_a_b + n_a_b);
        end
        measured_accel_outputs(index, :) = mean(recorded_accel_ouptputs);
        true_accel_outputs(index, :) = transpose(f_bi_b);
        index = index + 1;
    end
end


%% solve the calibration problem for three channels of accel
H = [ones(size(true_accel_outputs, 1), 1), true_accel_outputs];
pesudu_inverse = inv(transpose(H) * H) * transpose(H);

% bias vector and M matrix
calibration_matrix = pesudu_inverse * measured_accel_outputs;
bias_calibration = calibration_matrix(1, :);
M_calibration = calibration_matrix(2:4, :);


% Compute percentage error for bias
bias_error_percent = 100 * (bias_calibration - b_a_b') ./ b_a_b';

% Compute percentage error for each element of M
M_error_percent = 100 * (M_calibration - M) ./ M;

% Display results
disp('Accel Bias estimation error in percentage:');
disp(array2table(bias_error_percent, 'VariableNames', {'X', 'Y', 'Z'}));

disp('Accel M matrix estimation error in percentage:');
disp(array2table(M_error_percent, ...
    'VariableNames', {'X', 'Y', 'Z'}, ...
    'RowNames', {'X_row', 'Y_row', 'Z_row'}));
