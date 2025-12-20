clc;
close all;

% Create result directory if it doesn't exist
if ~exist('result', 'dir')
    mkdir('result');
end

% Plot Latitude
figure;
plot(trueLLA.lat, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.lat, 'r--', 'LineWidth', 1.5);
grid on;
title('Latitude Comparison');
xlabel('Time Step');
ylabel('Latitude [deg]');
legend('True Latitude', 'Estimated Latitude');
saveas(gcf, 'result/latitude_comparison.png');

% Plot Longitude
figure;
plot(trueLLA.lon, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.lon, 'r--', 'LineWidth', 1.5);
grid on;
title('Longitude Comparison');
xlabel('Time Step');
ylabel('Longitude [deg]');
legend('True Longitude', 'Estimated Longitude');
saveas(gcf, 'result/longitude_comparison.png');

% Plot Altitude
figure;
plot(trueLLA.alt, 'b-', 'LineWidth', 1.5);
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
plot(trueVNED.VN, 'b-', 'LineWidth', 1.5);
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
plot(trueVNED.VE, 'b-', 'LineWidth', 1.5);
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
plot(trueVNED.VD, 'b-', 'LineWidth', 1.5);
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
plot(trueEuler.phi, 'b-', 'LineWidth', 1.5);
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
plot(trueEuler.theta, 'b-', 'LineWidth', 1.5);
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
plot(trueEuler.psi, 'b-', 'LineWidth', 1.5);
hold on;
plot(out.si, 'r--', 'LineWidth', 1.5);
grid on;
title('Yaw Angle Comparison (\psi)');
xlabel('Time Step');
ylabel('Yaw Angle [rad]');
legend('True \psi', 'Estimated \psi');
saveas(gcf, 'result/yaw_angle.png');
