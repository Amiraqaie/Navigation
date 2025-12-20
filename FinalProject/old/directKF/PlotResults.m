clc;
close all;

% Create result directory if it doesn't exist
if ~exist('result', 'dir')
    mkdir('result');
end

%% Plot Estimated Resultsظ÷
% Plot Latitude
figure;
plot(trueData100Hz.lat, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.lat, 'r--', 'LineWidth', 1.5);
grid on;
title('Latitude Comparison');
xlabel('Time Step');
ylabel('Latitude [rad]');
legend('True Latitude', 'Estimated Latitude');
saveas(gcf, 'result/latitude_comparison.png');

% Plot Longitude
figure;
plot(trueData100Hz.lon, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.lon, 'r--', 'LineWidth', 1.5);
grid on;
title('Longitude Comparison');
xlabel('Time Step');
ylabel('Longitude [rad]');
legend('True Longitude', 'Estimated Longitude');
saveas(gcf, 'result/longitude_comparison.png');

% Plot Altitude
figure;
plot(trueData100Hz.alt, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.alt, 'r--', 'LineWidth', 1.5);
grid on;
title('Altitude Comparison');
xlabel('Time Step');
ylabel('Altitude [m]');
legend('True Altitude', 'Estimated Altitude');
saveas(gcf, 'result/altitude_comparison.png');

% Plot Velocity - North
figure;
plot(trueData100Hz.vn, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.vn, 'r--', 'LineWidth', 1.5);
grid on;
title('North Velocity Comparison');
xlabel('Time Step');
ylabel('Velocity North [m/s]');
legend('True VN', 'Estimated VN');
saveas(gcf, 'result/velocity_north.png');

% Plot Velocity - East
figure;
plot(trueData100Hz.ve, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.ve, 'r--', 'LineWidth', 1.5);
grid on;
title('East Velocity Comparison');
xlabel('Time Step');
ylabel('Velocity East [m/s]');
legend('True VE', 'Estimated VE');
saveas(gcf, 'result/velocity_east.png');

% Plot Velocity - Down
figure;
plot(trueData100Hz.vd, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.vd, 'r--', 'LineWidth', 1.5);
grid on;
title('Down Velocity Comparison');
xlabel('Time Step');
ylabel('Velocity Down [m/s]');
legend('True VD', 'Estimated VD');
saveas(gcf, 'result/velocity_down.png');

% Plot Roll (phi)
figure;
plot(trueData100Hz.roll, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.phi, 'r--', 'LineWidth', 1.5);
grid on;
title('Roll Angle Comparison (\phi)');
xlabel('Time Step');
ylabel('Roll Angle [rad]');
legend('True \phi', 'Estimated \phi');
saveas(gcf, 'result/roll_angle.png');

% Plot Pitch (theta)
figure;
plot(trueData100Hz.pitch, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.theta, 'r--', 'LineWidth', 1.5);
grid on;
title('Pitch Angle Comparison (\theta)');
xlabel('Time Step');
ylabel('Pitch Angle [rad]');
legend('True \theta', 'Estimated \theta');
saveas(gcf, 'result/pitch_angle.png');

% Plot Yaw (psi / si)
figure;
plot(trueData100Hz.yaw, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.si, 'r--', 'LineWidth', 1.5);
grid on;
title('Yaw Angle Comparison (\psi)');
xlabel('Time Step');
ylabel('Yaw Angle [rad]');
legend('True \psi', 'Estimated \psi');
saveas(gcf, 'result/yaw_angle.png');

% Accel Bias X
figure;
plot(out.b_ax, 'b-', 'LineWidth', 1.0);
grid on;
title('Accel Bias X Direction');
xlabel('Time Step');
ylabel('Accel Bias X [m/s^2]');
saveas(gcf, 'result/accel_bias_x.png');

% Accel Bias Y
figure;
plot(out.b_ay, 'b-', 'LineWidth', 1.0);
grid on;
title('Accel Bias Y Direction');
xlabel('Time Step');
ylabel('Accel Bias Y [m/s^2]');
saveas(gcf, 'result/accel_bias_y.png');

% Accel Bias Z
figure;
plot(out.b_az, 'b-', 'LineWidth', 1.0);
grid on;
title('Accel Bias Z Direction');
xlabel('Time Step');
ylabel('Accel Bias Z [m/s^2]');
saveas(gcf, 'result/accel_bias_z.png');

% Gyro Bias X
figure;
plot(out.b_gx, 'b-', 'LineWidth', 1.0);
grid on;
title('Gyro Bias X Direction');
xlabel('Time Step');
ylabel('Gyro Bias X [rad/s]');
saveas(gcf, 'result/gyro_bias_x.png');

% Gyro Bias Y
figure;
plot(out.b_gy, 'b-', 'LineWidth', 1.0);
grid on;
title('Gyro Bias Y Direction');
xlabel('Time Step');
ylabel('Gyro Bias Y [rad/s]');
saveas(gcf, 'result/gyro_bias_y.png');

% Gyro Bias Z
figure;
plot(out.b_gz, 'b-', 'LineWidth', 1.0);
grid on;
title('Gyro Bias Z Direction');
xlabel('Time Step');
ylabel('Gyro Bias Z [rad/s]');
saveas(gcf, 'result/gyro_bias_z.png');

%% Plot Errors of Estimations
% Error of North Direction
figure;
plot(out.error_north, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('North Direction Error');
xlabel('Time Step');
ylabel('North Error [meter]');
saveas(gcf, 'result/north_error.png');

% Error of East Direction
figure;
plot(out.error_east, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('East Direction Error');
xlabel('Time Step');
ylabel('East Error [meter]');
saveas(gcf, 'result/east_error.png');

% Error of Down Direction
figure;
plot(out.error_down, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Down Direction Error');
xlabel('Time Step');
ylabel('Down Error [meter]');
saveas(gcf, 'result/down_error.png');

% Error of Latitude
figure;
plot(out.error_lat, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Latitude Error');
xlabel('Time Step');
ylabel('Latitude Error [rad]');
saveas(gcf, 'result/latitude_error.png');

% Error of Longitude
figure;
plot(out.error_lon, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Longitude Error');
xlabel('Time Step');
ylabel('Longitude Error [rad]');
saveas(gcf, 'result/longitude_error.png');

% Error of Altitude
figure;
plot(out.error_alt, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Altitude Error');
xlabel('Time Step');
ylabel('Altitude Error [meter]');
saveas(gcf, 'result/altitude_error.png');

% Error of North Veclocity
figure;
plot(out.error_vn, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('North Veclocity Error');
xlabel('Time Step');
ylabel('North Veclocity Error [m/s]');
saveas(gcf, 'result/north_velocity_error.png');

% Error of East Veclocity
figure;
plot(out.error_ve, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('East Veclocity Error');
xlabel('Time Step');
ylabel('East Veclocity Error [m/s]');
saveas(gcf, 'result/east_velocity_error.png');

% Error of Down Veclocity
figure;
plot(out.error_vd, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Down Veclocity Error');
xlabel('Time Step');
ylabel('Down Veclocity Error [m/s]');
saveas(gcf, 'result/down_velocity_error.png');

% Error of Roll Angle
figure;
plot(rad2deg(out.error_phi), 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Roll Angle Error');
xlabel('Time Step');
ylabel('Roll Angle [degree]');
saveas(gcf, 'result/roll_angle_error.png');

% Error of Pitch Angle
figure;
plot(rad2deg(out.error_theta), 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Pitch Angle Error');
xlabel('Time Step');
ylabel('Pitch Angle [degree]');
saveas(gcf, 'result/pitch_angle_error.png');

% Error of Yaw Angle
figure;
plot(rad2deg(out.error_si), 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Yaw Angle Error');
xlabel('Time Step');
ylabel('Yaw Angle [degree]');
saveas(gcf, 'result/yaw_angle_error.png');

%% Plot Covariace of Estimations
% Covariace of Latitude
figure;
semilogy(out.p_lat, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Latitude Covariace');
xlabel('Time Step');
ylabel('Latitude Covariace [rad^2]');
saveas(gcf, 'result/latitude_covariance.png');

% Covariace of Longitude
figure;
semilogy(out.p_lon, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Longitude Covariace');
xlabel('Time Step');
ylabel('Longitude Covariace [rad^2]');
saveas(gcf, 'result/longitude_covariance.png');

% Covariace of Altitude
figure;
semilogy(out.p_alt, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Altitude Covariace');
xlabel('Time Step');
ylabel('Altitude Covariace [meter^2]');
saveas(gcf, 'result/altitude_covariance.png');

% Covariace of North Veclocity
figure;
semilogy(out.p_vn, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('North Veclocity Covariace');
xlabel('Time Step');
ylabel('North Veclocity Covariace [m^2/s^2]');
saveas(gcf, 'result/north_velocity_covariance.png');

% Covariace of East Veclocity
figure;
semilogy(out.p_ve, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('East Veclocity Covariace');
xlabel('Time Step');
ylabel('East Veclocity Covariace [m^2/s^2]');
saveas(gcf, 'result/east_velocity_covariance.png');

% Covariace of Down Veclocity
figure;
semilogy(out.p_vd, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Down Veclocity Covariace');
xlabel('Time Step');
ylabel('Down Veclocity Covariace [m^2/s^2]');
saveas(gcf, 'result/down_velocity_covariance.png');

% Covariace of Roll Angle
figure;
semilogy(out.p_phi, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Roll Angle Covariace');
xlabel('Time Step');
ylabel('Roll Angle Covariace [rad^2]');
saveas(gcf, 'result/roll_angle_variance.png');

% Covariace of Pitch Angle
figure;
semilogy(out.p_theta, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Pitch Angle Covariace');
xlabel('Time Step');
ylabel('Pitch Angle Covariace [rad^2]');
saveas(gcf, 'result/pitch_angle_covariance.png');

% Covariace of Yaw Angle
figure;
semilogy(out.p_si, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Yaw Angle Covariace');
xlabel('Time Step');
ylabel('Yaw Angle Covariace [rad^2]');
saveas(gcf, 'result/yaw_angle_covariance.png');

% Covariace of Accel Bias X
figure;
semilogy(out.p_b_ax, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Accel Bias X Covariace');
xlabel('Time Step');
ylabel('Accel Bias X Covariace [m^2/s^4]');
saveas(gcf, 'result/aceel_bias_x_covariance.png');

% Covariace of Accel Bias Y
figure;
semilogy(out.p_b_ay, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Accel Bias Y Covariace');
xlabel('Time Step');
ylabel('Accel Bias Y Covariace [m^2/s^4]');
saveas(gcf, 'result/aceel_bias_y_covariance.png');

% Covariace of Accel Bias Z
figure;
semilogy(out.p_b_gz, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Accel Bias Z Covariace');
xlabel('Time Step');
ylabel('Accel Bias Z Covariace [m^2/s^4]');
saveas(gcf, 'result/aceel_bias_z_covariance.png');

% Covariace of Gyro Bias X
figure;
semilogy(out.p_b_gx, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Gyro Bias X Covariace');
xlabel('Time Step');
ylabel('Gyro Bias X Covariace [rad^2/s^2]');
saveas(gcf, 'result/gyro_bias_x_covariance.png');

% Covariace of Gyro Bias Y
figure;
semilogy(out.p_b_gy, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Gyro Bias Y Covariace');
xlabel('Time Step');
ylabel('Gyro Bias Y Covariace [rad^2/s^2]');
saveas(gcf, 'result/gyro_bias_y_covariance.png');

% Covariace of Gyro Bias Z
figure;
semilogy(out.p_b_gz, 'color', [0.5, 0.5, 0.841], 'LineWidth', 1.0);
grid on;
title('Gyro Bias Z Covariace');
xlabel('Time Step');
ylabel('Gyro Bias Z Covariace [rad^2/s^2]');
saveas(gcf, 'result/gyro_bias_z_covariance.png');

%% Report RMS Results
RMS.RMS_LATITUDE_ERROR = rms(out.error_lat);
RMS.RMS_LONGITUDE_ERROR = rms(out.error_lon);
RMS.RMS_ALTITUDE_ERROR = rms(out.error_alt);
RMS.RMS_VN_ERROR = rms(out.error_vn);
RMS.RMS_VE_ERROR = rms(out.error_ve);
RMS.RMS_VD_ERROR = rms(out.error_vd);
RMS.RMS_ROLL_ANGLE_ERROR = rad2deg(rms(out.error_phi));
RMS.RMS_PITCH_ANGLE_ERROR = rad2deg(rms(out.error_theta));
RMS.RMS_YAW_ANGLE_ERROR = rad2deg(rms(out.error_si));
RMS.RMS_NORTH_ERROR = rms(out.error_north);
RMS.RMS_EAST_ERROR = rms(out.error_east);
RMS.RMS_DOWN_ERROR = rms(out.error_down);

disp(RMS);
