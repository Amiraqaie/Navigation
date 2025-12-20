function state_derivative = state_derivative(x_n, Wbi_b, Fbi_b)
    lon = x_n(1,1);
    lat = x_n(2,1);
    h = x_n(3,1);
    vn = x_n(4,1);
    ve = x_n(5,1);
    vd = x_n(6,1);
    Vbe_n = [vn;
             ve;
             vd];
    phi = x_n(7,1);
    theta = x_n(8,1);
    si = x_n(9,1);

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


    Wei_e = [0;
            0;
            wei];
    Wne_n = [ve / (Rn + h);
            -vn / (Rm + h);
            -ve / (Rn + h) * tan(lat)];
    T_ne = [-sin(lat) * cos(lon),    -sin(lat) * sin(lon),   cos(lat);
            -sin(lon),              cos(lon),               0;      
            -cos(lat) * cos(lon),   -cos(lat) * sin(lon),   -sin(lat)];
    T_bn = [cos(si) * cos(theta),                                   sin(si) * cos(theta),                                   -sin(theta);
            cos(si) * sin(theta) * sin(phi) - sin(si) * cos(phi),   sin(si) * sin(theta) * sin(phi) + cos(si) * cos(phi),   cos(theta) * sin(phi);
            cos(si) * sin(theta) * cos(phi) + sin(si) * sin(phi),   sin(si) * sin(theta) * cos(phi) - cos(si) * sin(phi),   cos(theta) * cos(phi)];
    Wbn_b = Wbi_b - T_bn * Wne_n - T_bn * T_ne * Wei_e;
    Dn_Vbe_n = T_bn' * Fbi_b + g - 2 * T_ne * skew_symetric(Wei_e) * T_ne' * Vbe_n - skew_symetric(Wne_n) * Vbe_n;
    state_derivative = [ve / ((Rn + h) * cos(lat));
                        vn / (Rm + h);
                        -vd;
                        Dn_Vbe_n;
                        [1, sin(phi) * tan(theta),  cos(phi) * tan(theta);
                        0,  cos(phi),               -sin(phi);
                        0,  sin(phi) / cos(theta),  cos(phi) / cos(theta)] * Wbn_b];

end

