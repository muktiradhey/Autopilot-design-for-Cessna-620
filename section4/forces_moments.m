% forces_moments.m
%   Computes the forces and moments acting on the airframe.
%
%   Output is
%       F     - forces        [3x1]
%       M     - moments       [3x1]
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame

function out = forces_moments(x, delta, wind, P)

    % relabel the states
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);   % fixed: read from state
    v       = x(5);   % fixed: read from state
    w       = x(6);   % fixed: read from state
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);

    % relabel the controls
    delta_e = delta(1);   % fixed: read from delta
    delta_a = delta(2);   % fixed: read from delta
    delta_r = delta(3);   % fixed: read from delta
    delta_t = delta(4);   % fixed: read from delta

    % relabel wind inputs
    w_ns = wind(1); % steady wind - North
    w_es = wind(2); % steady wind - East
    w_ds = wind(3); % steady wind - Down
    u_wg = wind(4); % gust along body x-axis
    v_wg = wind(5); % gust along body y-axis
    w_wg = wind(6); % gust along body z-axis

    % wind in NED for output
    w_n = w_ns;
    w_e = w_es;
    w_d = w_ds;

    % rotate steady wind from inertial to body frame
    R_bi = [...
        cos(theta)*cos(psi), ...
        cos(theta)*sin(psi), ...
        -sin(theta);...
        sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), ...
        sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), ...
        sin(phi)*cos(theta);...
        cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), ...
        cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), ...
        cos(phi)*cos(theta)];
    wind_body = R_bi * [w_ns; w_es; w_ds];

    % relative wind in body frame
    ur = u - (wind_body(1) + u_wg);
    vr = v - (wind_body(2) + v_wg);
    wr = w - (wind_body(3) + w_wg);

    % compute air data
    Va    = sqrt(ur^2 + vr^2 + wr^2) + 0.001;
    alpha = atan2(wr, ur);
    beta  = asin(vr / Va);

    % gravity force in body frame
    Fg = P.mass * P.gravity * ...
        [-sin(theta);
          sin(phi)*cos(theta);
          cos(phi)*cos(theta)];

    % dynamic pressure
    qbar = 0.5 * P.rho * Va^2;

    % lift coefficient
    CL = P.C_L_0 + P.C_L_alpha * alpha ...
       + P.C_L_q * (P.c/(2*Va)) * q ...
       + P.C_L_delta_e * delta_e;

    % drag coefficient
    CD = P.C_D_0 + P.C_D_alpha * alpha ...
       + P.C_D_delta_e * delta_e;

    % aerodynamic forces in body frame (Eq. 4.19)
    Fx_aero = qbar * P.S_wing * (-CD*cos(alpha) + CL*sin(alpha));
    Fz_aero = qbar * P.S_wing * (-CD*sin(alpha) - CL*cos(alpha));

    % side force
    CY = P.C_Y_0 + P.C_Y_beta * beta ...
       + P.C_Y_p * (P.b/(2*Va)) * p ...
       + P.C_Y_r * (P.b/(2*Va)) * r ...
       + P.C_Y_delta_a * delta_a ...
       + P.C_Y_delta_r * delta_r;
    Fy_aero = qbar * P.S_wing * CY;

    % thrust (Eq. 4.18)
    % CORRECT for Cessna piston engine
    T = P.T_max * delta_t;

    % total forces
    Force = Fg + [Fx_aero + T; Fy_aero; Fz_aero];

    % roll moment coefficient
    Cl = P.C_ell_0 ...
       + P.C_ell_beta * beta ...
       + P.C_ell_p * (P.b/(2*Va)) * p ...
       + P.C_ell_r * (P.b/(2*Va)) * r ...
       + P.C_ell_delta_a * delta_a ...
       + P.C_ell_delta_r * delta_r;

    % pitch moment coefficient
    Cm = P.C_m_0 ...
       + P.C_m_alpha * alpha ...
       + P.C_m_q * (P.c/(2*Va)) * q ...
       + P.C_m_delta_e * delta_e;

    % yaw moment coefficient
    Cn = P.C_n_0 ...
       + P.C_n_beta * beta ...
       + P.C_n_p * (P.b/(2*Va)) * p ...
       + P.C_n_r * (P.b/(2*Va)) * r ...
       + P.C_n_delta_a * delta_a ...
       + P.C_n_delta_r * delta_r;

    % moments
    L = qbar * P.S_wing * P.b * Cl;
    M = qbar * P.S_wing * P.c * Cm;
    N = qbar * P.S_wing * P.b * Cn;

    Torque = [L; M; N];

    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];
end
