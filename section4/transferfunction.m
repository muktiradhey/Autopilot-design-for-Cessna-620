function [T_phi_delta_a,...
          T_chi_phi,...
          T_theta_delta_e,...
          T_h_theta,...
          T_h_Va,...
          T_Va_delta_t,...
          T_Va_theta,...
          T_v_delta_r]...
          = transferfunction(x_trim, u_trim, P)
% =========================================================
% AE700 Project - Part 4(vi)
% Function: compute_tf_model.m
%
% PURPOSE:
%   Returns all transfer functions from Section 5.4
%   of Beard & McLain using trim values.
%
% INPUTS:
%   x_trim - [12x1] trim state
%   u_trim - [4x1]  trim inputs
%   P      - aircraft parameter struct
%
% OUTPUTS:
%   T_phi_delta_a   - phi/delta_a    (Eq. 5.26)
%   T_chi_phi       - chi/phi        (Eq. 5.27)
%   T_theta_delta_e - theta/delta_e  (Eq. 5.29)
%   T_h_theta       - h/theta        (Eq. 5.31)
%   T_h_Va          - h/Va           (Eq. 5.32)
%   T_Va_delta_t    - Va/delta_t     (Eq. 5.36)
%   T_Va_theta      - Va/theta       (Eq. 5.36)
%   T_v_delta_r     - beta/delta_r   (Eq. 5.28)
% =========================================================

    % ---- extract trim values ----
    Va_star      = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2);
    alpha_star   = atan2(x_trim(6), x_trim(4));
    theta_star   = x_trim(8);
    delta_e_star = u_trim(1);
    delta_t_star = u_trim(4);

    % =========================================================
    % LATERAL TRANSFER FUNCTIONS
    % =========================================================

    % --- phi/delta_a = a_phi2 / [s(s + a_phi1)]  (Eq. 5.26) ---
    a_phi1 = -0.5 * P.rho * Va_star^2 * P.S_wing * P.b ...
              * P.C_ell_p * (P.b / (2*Va_star)) / P.Jx;
    a_phi2 =  0.5 * P.rho * Va_star^2 * P.S_wing * P.b ...
              * P.C_ell_delta_a / P.Jx;

    T_phi_delta_a = tf(a_phi2, [1, a_phi1, 0]);

    % --- chi/phi = (g/Va) / s  (Eq. 5.27) ---
    T_chi_phi = tf(P.gravity / Va_star, [1, 0]);

    % --- beta/delta_r = a_beta2 / (s + a_beta1)  (Eq. 5.28) ---
    a_beta1 = -(P.rho * Va_star * P.S_wing / (2*P.mass)) * P.C_Y_beta;
    a_beta2 =  (P.rho * Va_star * P.S_wing / (2*P.mass)) * P.C_Y_delta_r;

    T_v_delta_r = tf(a_beta2, [1, a_beta1]);

    % =========================================================
    % LONGITUDINAL TRANSFER FUNCTIONS
    % =========================================================

    % --- theta/delta_e = a_theta3/(s^2 + a_theta1*s + a_theta2)  (Eq. 5.29) ---
    a_theta1 = -(P.rho * Va_star^2 * P.S_wing * P.c * P.C_m_q) ...
                * (P.c / (2*Va_star)) / (2*P.Jy);
    a_theta2 = -(P.rho * Va_star^2 * P.S_wing * P.c * P.C_m_alpha) ...
                / (2*P.Jy);
    a_theta3 =  (P.rho * Va_star^2 * P.S_wing * P.c * P.C_m_delta_e) ...
                / (2*P.Jy);

    T_theta_delta_e = tf(a_theta3, [1, a_theta1, a_theta2]);

    % --- h/theta = Va/s  (Eq. 5.31) ---
    T_h_theta = tf(Va_star, [1, 0]);

    % --- h/Va = theta*/s  (Eq. 5.32) ---
    T_h_Va = tf(theta_star, [1, 0]);

    % --- Va/delta_t = a_V2/(s + a_V1)  (Eq. 5.36) ---
    a_V1 = -dT_dVa(P, Va_star, delta_t_star) / P.mass ...
           + (P.rho * Va_star * P.S_wing / P.mass) ...
           * (P.C_D_0 + P.C_D_alpha*alpha_star + P.C_D_delta_e*delta_e_star);
    a_V2 =  dT_ddelta_t(P, Va_star, delta_t_star) / P.mass;
    a_V3 =  P.gravity * cos(theta_star - alpha_star);

    T_Va_delta_t = tf(a_V2,  [1, a_V1]);

    % --- Va/theta = -a_V3/(s + a_V1)  (Eq. 5.36) ---
    T_Va_theta = tf(-a_V3, [1, a_V1]);

    % ---- print summary ----
    fprintf('\n===== Transfer Functions at Trim =====\n');
    fprintf('Va* = %.4f m/s, alpha* = %.4f deg\n', Va_star, alpha_star*180/pi);
    fprintf('\n--- Lateral ---\n');
    fprintf('a_phi1=%.4f, a_phi2=%.4f\n', a_phi1, a_phi2);
    fprintf('a_beta1=%.4f, a_beta2=%.4f\n', a_beta1, a_beta2);
    fprintf('\n--- Longitudinal ---\n');
    fprintf('a_theta1=%.4f, a_theta2=%.4f, a_theta3=%.4f\n', a_theta1, a_theta2, a_theta3);
    fprintf('a_V1=%.4f, a_V2=%.4f, a_V3=%.4f\n', a_V1, a_V2, a_V3);
    fprintf('=======================================\n\n');

end

% =========================================================
% returns derivative of thrust with respect to Va
% dT/dVa  (Eq. 4.18 linearized — linear model: T = T_max*delta_t)
% For linear thrust model dT/dVa = 0
% =========================================================
function dThrust = dT_dVa(P, Va, delta_t)
    % linear thrust model: T = T_max * delta_t
    % thrust does not depend on Va => dT/dVa = 0
    dThrust = 0;

    % --- if using Eq. 4.18 propulsion model, use this instead ---
    % dThrust = -P.rho * P.S_prop * P.C_prop * Va;
end

% =========================================================
% returns derivative of thrust with respect to delta_t
% dT/d(delta_t)  (linearized around trim)
% =========================================================
function dThrust = dT_ddelta_t(P, Va, delta_t)
    % linear thrust model: T = T_max * delta_t
    % dT/d(delta_t) = T_max
    dThrust = P.T_max;

    % --- if using Eq. 4.18 propulsion model, use this instead ---
    % dThrust = P.rho * P.S_prop * P.C_prop * P.k_motor^2 * delta_t;
end