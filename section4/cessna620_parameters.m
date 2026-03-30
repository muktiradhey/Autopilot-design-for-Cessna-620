function P = cessna620_parameters()
% =========================================================
% Cessna 620 Aircraft Parameters
% Units: SI (m, kg, m/s, rad)
% Converted from Imperial (ft, slugs, ft/s)
% =========================================================

%% Basic constants
P.gravity = 9.81;       % m/s^2
P.rho     = 1.2256;     % kg/m^3 (sea level standard atmosphere)

%% Mass and geometry
% 15000 lb = 6803.9 kg
P.mass   = 15000 * 0.453592;   % kg
% 1 ft^2 = 0.0929 m^2
P.S_wing = 340   * 0.092903;   % m^2  (wing area)
% 1 ft = 0.3048 m
P.b      = 55.1  * 0.3048;     % m    (wingspan)
P.c      = 6.58  * 0.3048;     % m    (mean chord)

%% Inertia
% 1 slug*ft^2 = 1.35582 kg*m^2
P.Jx  = 64811 * 1.35582;       % kg*m^2
P.Jy  = 17300 * 1.35582;       % kg*m^2
P.Jz  = 64543 * 1.35582;       % kg*m^2
P.Jxz = 0;                      % kg*m^2

%% ===== LONGITUDINAL COEFFICIENTS =====
% Dimensionless — no conversion needed
P.C_L_0       =  0.48;
P.C_L_alpha   =  5.55;
P.C_L_q       =  7.5;
P.C_L_delta_e =  0.58;

P.C_D_0       =  0.0322;
P.C_D_alpha   =  0.269;
P.C_D_delta_e =  0.0;

P.C_m_0       =  0.06;
P.C_m_alpha   = -1.18;
P.C_m_q       = -22.4;
P.C_m_delta_e = -1.73;

%% ===== LATERAL COEFFICIENTS =====
% Dimensionless — no conversion needed
P.C_Y_0       =  0;
P.C_Y_beta    = -0.883;
P.C_Y_p       = -0.227;
P.C_Y_r       =  0.448;
P.C_Y_delta_a =  0;
P.C_Y_delta_r =  0.2;

P.C_ell_0       =  0;
P.C_ell_beta    = -0.1381;
P.C_ell_p       = -0.566;
P.C_ell_r       =  0.1166;
P.C_ell_delta_a = -0.1776;
P.C_ell_delta_r =  0.02;

P.C_n_0       =  0;
P.C_n_beta    =  0.1739;
P.C_n_p       = -0.0501;
P.C_n_r       = -0.2;
P.C_n_delta_a =  0.0194;
P.C_n_delta_r = -0.1054;

%% ===== PROPULSION =====
% 1 lb = 4.44822 N
P.T_max   = 2500 * 4.44822;    % N (max thrust)
% 1 ft^2 = 0.0929 m^2
P.S_prop  = 2.0  * 0.092903;   % m^2 (propeller disk area)
P.C_prop  = 1.0;                % dimensionless
% 1 ft/s = 0.3048 m/s
P.k_motor = 40   * 0.3048;     % m/s per unit throttle
P.k_T_p   = 0;
P.k_Omega = 0;

%% ===== STALL MODEL =====
P.M      = 50;                  % dimensionless
P.alpha0 = 0.4712;              % rad (~27 deg)
P.e      = 0.9;                 % Oswald efficiency

%% ===== TRIM AIRSPEED =====
% 1 ft/s = 0.3048 m/s
% 200 ft/s = 60.96 m/s
P.Va0 = 69.6690;          % m/s (~61 m/s, typical Cessna cruise)

%% ===== INITIAL CONDITIONS =====
P.pn0    =  0;                  % m
P.pe0    =  0;                  % m
% 200 ft = 60.96 m
P.pd0    = -500;       % m  (altitude = 60.96 m)
P.u0     =  69.5094;              % m/s
P.v0     =  0;                  % m/s
P.w0     =  4.7130;                  % m/s
P.phi0   =  33.5486*pi/180;                  % rad
P.theta0 =  3.2343*pi/180;                  % rad
P.psi0   =  0;                  % rad
P.p0     =  -0.0053;                  % rad/s
P.q0     =  0.0515;                  % rad/s
P.r0     =  0.0777;                  % rad/s

% %% ===== PRINT SUMMARY =====
% fprintf('Cessna 620 Parameters (SI units):\n');
% fprintf('  mass    = %.2f kg\n',  P.mass);
% fprintf('  S_wing  = %.4f m^2\n', P.S_wing);
% fprintf('  b       = %.4f m\n',   P.b);
% fprintf('  c       = %.4f m\n',   P.c);
% fprintf('  Va0     = %.2f m/s\n', P.Va0);
% fprintf('  T_max   = %.2f N\n',   P.T_max);
% fprintf('  pd0     = %.2f m (altitude = %.2f m)\n', P.pd0, -P.pd0);

end