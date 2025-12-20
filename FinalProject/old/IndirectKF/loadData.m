clear;
close all;
clc;


load('../../data/trueData100Hz.mat');
load('../../data/imuData100Hz.mat');
load('../../data/gpsData10Hz.mat');

% define sampling time and Ts of kalman filter
filter_Ts = 0.01;
imu_sampling_time = 0.01;
gps_sampling_time = 0.1;

% Calculate time series of data
N = length(gpsData10Hz.lat);
gpsData10Hz.time = (0:N-1)' * gps_sampling_time;

N = length(imuData100Hz.ax);
imuData100Hz.time = (0:N-1)' * imu_sampling_time;

N = length(trueData100Hz.lat);
trueData100Hz.time = (0:N-1)' * filter_Ts;

% GPS data (10 Hz): lat, lon
gpsSim.time = gpsData10Hz.time;
gpsSim.signals.values = [gpsData10Hz.lon(:), gpsData10Hz.lat(:), gpsData10Hz.alt(:), gpsData10Hz.vn(:), gpsData10Hz.ve(:), gpsData10Hz.vd(:)];
gpsSim.signals.dimensions = 6;

% IMU data (100 Hz): ax, ay, yawRate
imuSim.time = imuData100Hz.time;
imuSim.signals.values = [imuData100Hz.ax(:), imuData100Hz.ay(:), imuData100Hz.az(:), imuData100Hz.wx(:), imuData100Hz.wy(:), imuData100Hz.wz(:)];
imuSim.signals.dimensions = 6;

% Ground truth data (100 Hz): lat, lon, yaw
trueSim.time = trueData100Hz.time;
trueSim.signals.values = [trueData100Hz.lon(:), trueData100Hz.lat(:), trueData100Hz.alt(:), trueData100Hz.vn(:), trueData100Hz.ve(:), trueData100Hz.vd(:), trueData100Hz.roll(:), trueData100Hz.pitch(:), trueData100Hz.yaw(:)];
trueSim.signals.dimensions = 9;

% Initial Condition
row_trueData = trueSim.signals.values(1, :);
row_GpsData = gpsSim.signals.values(1, :);
lon_0 = row_GpsData(1);  
lat_0 = row_GpsData(2);
h_0 = row_GpsData(3);
vn_0  = 0;
ve_0  = 0;
vd_0  = 0;
phi_0 = 0;
theta_0 = 0;
si_0 = 0;

% Scale factor definition
p_0 = 100 * eye(9);
x_0 = [lon_0;lat_0;h_0;vn_0;ve_0;vd_0;phi_0;theta_0;si_0];
dx_0 = zeros(length(x_0), 1);
u_0 = imuSim.signals.values(1, :);

% Best tunned Q and R parameters based on computeImuProcessNoise and computeGpsMeasurementNoise
Qk = diag([1e-11, 1e-11, 1e-11, 4e-5,  4e-5,  4e-5, 1.2e-5, 1.2e-5, 1.2e-5]);
Rk = diag([(7.84e-5)^2, (7.84e-5)^2, (2.0)^2, (0.1)^2, (0.1)^2, (0.1)^2]);

% Load jacobian file
%calculateSystemJacobian;
calculateSystemJacobianSimple;
F_0 = Fk(x_0, imuSim.signals.values(1, 1:3)', imuSim.signals.values(1, 4:6)', filter_Ts);
B_0 = Bk(x_0, imuSim.signals.values(1, 1:3)', imuSim.signals.values(1, 4:6)', filter_Ts);
