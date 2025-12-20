function IMU_NOISE = computeImuNoise(imuData100Hz, N)
    % Default N to 100000 if not provided
    if nargin < 4
        N = 100000;
    end

    % === Extract IMU data ===
    gyro_data = [imuData100Hz.wx(1:N), imuData100Hz.wy(1:N), imuData100Hz.wz(1:N)];
    accel_data = [imuData100Hz.ax(1:N), imuData100Hz.ay(1:N), imuData100Hz.az(1:N)];

    % === Zero-mean signals ===
    gyro_zero_mean = gyro_data - mean(gyro_data, 1);
    accel_zero_mean = accel_data - mean(accel_data, 1);

    % === Standard deviation and continuous-time variance ===
    gyro_std = std(gyro_zero_mean);                      % rad/s
    accel_std = std(accel_zero_mean);                    % m/s²

    gyro_var_continuous  = (gyro_std .^ 2) ;     % rad²/s³
    accel_var_continuous = (accel_std .^ 2) ;    % (m/s²)²/s

    % === Build diagonal Qk matrix ===
    % For example: 3 accel + 3 gyro
    IMU_NOISE = diag([accel_var_continuous, gyro_var_continuous]);

end
