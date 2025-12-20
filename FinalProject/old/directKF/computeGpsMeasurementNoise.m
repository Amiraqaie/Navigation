function R = computeGpsMeasurementNoise(gpsData10Hz, trueData100Hz)
    % Ensure gpsData10Hz has the necessary fields
    requiredFields = {'lon', 'lat', 'alt', 've', 'vn', 'vd'};
    for i = 1:length(requiredFields)
        if ~isfield(gpsData10Hz, requiredFields{i})
            error(['Missing field: ', requiredFields{i}]);
        end
    end

    % Extract data
    lat_gps = rad2deg(gpsData10Hz.lat);
    lon_gps = rad2deg(gpsData10Hz.lon);
    alt_gps = gpsData10Hz.alt;
    ve_gps = gpsData10Hz.ve;
    vn_gps = gpsData10Hz.vn;
    vd_gps = gpsData10Hz.vd;

    lat_gt = rad2deg(trueData100Hz.lat(1:10:end));
    lon_gt = rad2deg(trueData100Hz.lon(1:10:end));
    alt_gt = trueData100Hz.alt(1:10:end);
    ve_gt = trueData100Hz.ve(1:10:end);
    vn_gt = trueData100Hz.vn(1:10:end);
    vd_gt = trueData100Hz.vd(1:10:end);
    

    % Compute zero-mean position and velocity
    pos_demeaned = [lat_gps, lon_gps, alt_gps] - [lat_gt, lon_gt, alt_gt];
    vel_demeaned = [ve_gps, vn_gps, vd_gps] - [ve_gt, vn_gt, vd_gt];
    
    pos_var = var(pos_demeaned);  % [σ²_east, σ²_north, σ²_up]
    vel_var = var(vel_demeaned);  % [σ²_ve, σ²_vn, σ²_vd]

    % Construct 6x6 measurement noise covariance matrix
    R = diag([pos_var, vel_var]);
end
