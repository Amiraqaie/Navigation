function R = computeGpsMeasurementNoise(gpsData10Hz, n)
    % Ensure gpsData10Hz has the necessary fields
    requiredFields = {'lon', 'lat', 'alt', 've', 'vn', 'vd'};
    for i = 1:length(requiredFields)
        if ~isfield(gpsData10Hz, requiredFields{i})
            error(['Missing field: ', requiredFields{i}]);
        end
    end

    % Extract data
    lat = rad2deg(gpsData10Hz.lat(1:n));
    lon = rad2deg(gpsData10Hz.lon(1:n));
    alt = gpsData10Hz.alt(1:n);
    ve = gpsData10Hz.ve(1:n);
    vn = gpsData10Hz.vn(1:n);
    vd = gpsData10Hz.vd(1:n);
    
    % Origin for ENU conversion (use average position)
    lat0 = mean(lat);
    lon0 = mean(lon);
    alt0 = mean(alt);

    % Convert to ENU (East-North-Up) in meters
    lla = [lat, lon, alt];
    origin = [lat0, lon0, alt0];
    enu = lla2enu(lla, origin, 'ellipsoid');

    % Extract position in ENU
    east  = enu(:, 1);
    north = enu(:, 2);
    up    = enu(:, 3);

    % Compute zero-mean position and velocity
    pos_demeaned = [east, north, up] - mean([east, north, up], 1);
    vel_demeaned = [ve, vn, vd] - mean([ve, vn, vd], 1);

    % Compute variances
    R_earth = 6371000;  % meters
    
    pos_var_m = var(pos_demeaned);  % [σ²_east, σ²_north, σ²_up]
    
    % Convert east and north variances from meters² to radians²
    pos_var_rad = zeros(size(pos_var_m));
    pos_var_rad(1) = pos_var_m(1) / (R_earth * cos(lat0))^2;  % lon variance (rad²)
    pos_var_rad(2) = pos_var_m(2) / R_earth^2;                 % lat variance (rad²)
    pos_var_rad(3) = pos_var_m(3);                             % up variance (m²) unchanged
    
    pos_var = pos_var_rad;
    vel_var = var(vel_demeaned);  % [σ²_ve, σ²_vn, σ²_vd]

    % Construct 6x6 measurement noise covariance matrix
    R = diag([pos_var, vel_var]);
end
