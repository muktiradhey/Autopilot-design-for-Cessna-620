function [A_lon, B_lon, A_lat, B_lat] = compute_ss_model(mav, x_trim, u_trim, P)
% =========================================================
% AE700 Project - Part 4(vii)
% Function: compute_ss_model.m.
%
% INPUTS:
%   mav    - Simulink model name (string) e.g. 'mavsim_trim'
%   x_trim - [12x1] trim state
%   u_trim - [4x1]  trim inputs
%   P      - aircraft parameter struct
%
% OUTPUTS:
%   A_lon - [5x5] longitudinal state matrix  (u,w,q,theta,h)
%   B_lon - [5x2] longitudinal input matrix  (delta_e, delta_t)
%   A_lat - [5x5] lateral state matrix       (v,p,r,phi,psi)
%   B_lat - [5x2] lateral input matrix       (delta_a, delta_r)
%
% METHOD:
%   Numerical Jacobian (Appendix F.4):
%   A = df/dx|trim,   B = df/du|trim
%   Each column computed by finite difference perturbation
%
% REF: Beard & McLain, Appendix F.4, Eqs. 5.43, 5.50
% =========================================================

    eps = 0.01;   % perturbation size for numerical Jacobian

    % convert trim state to Euler representation
    x_euler = euler_state(x_trim);
    delta    = u_trim;

    % ---- compute full 12x12 A and 12x4 B numerically ----
    A_full = df_dx(x_euler, delta, eps, P);
    B_full = df_du(x_euler, delta, eps, P);

    % ---- extract lateral sub-model (Eq. 5.43) ----
    % lateral states:  [v, p, r, phi, psi] = indices [5,10,12,7,9]
    % lateral inputs:  [delta_a, delta_r]  = indices [2, 4] of u_trim
    i_lat = [5, 10, 12, 7, 9];   % state indices
    i_u_lat = [2, 4];             % input indices

    A_lat = A_full(i_lat, i_lat);
    B_lat = B_full(i_lat, i_u_lat);

    % ---- extract longitudinal sub-model (Eq. 5.50) ----
    % longitudinal states: [u, w, q, theta, h] = indices [4,6,11,8,3]
    % longitudinal inputs: [delta_e, delta_t]  = indices [1, 4] of u_trim
    % note: h = -pd, so index 3 with sign flip handled below
    i_lon = [4, 6, 11, 8, 3];    % state indices (pd for h)
    i_u_lon = [1, 4];             % input indices

    A_lon = A_full(i_lon, i_lon);
    B_lon = B_full(i_lon, i_u_lon);

    % flip sign for altitude (h = -pd)
    A_lon(5,:) = -A_lon(5,:);
    A_lon(:,5) = -A_lon(:,5);
    B_lon(5,:) = -B_lon(5,:);

    % ---- print results ----
    fprintf('\n===== Longitudinal State-Space =====\n');
    fprintf('States: [u, w, q, theta, h]\n');
    fprintf('Inputs: [delta_e, delta_t]\n');
    fprintf('A_lon =\n'); disp(A_lon);
    fprintf('B_lon =\n'); disp(B_lon);
    fprintf('Eigenvalues of A_lon:\n');
    disp(eig(A_lon));

    fprintf('\n===== Lateral State-Space =====\n');
    fprintf('States: [v, p, r, phi, psi]\n');
    fprintf('Inputs: [delta_a, delta_r]\n');
    fprintf('A_lat =\n'); disp(A_lat);
    fprintf('B_lat =\n'); disp(B_lat);
    fprintf('Eigenvalues of A_lat:\n');
    disp(eig(A_lat));

end


% =========================================================
% euler_state
% convert quaternion state to Euler state
% quaternion state: [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r] (13x1)
% euler state:      [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r] (12x1)
% =========================================================
function x_euler = euler_state(x_quat)
    if length(x_quat) == 13
        % quaternion representation — convert to Euler
        e0 = x_quat(7);
        e1 = x_quat(8);
        e2 = x_quat(9);
        e3 = x_quat(10);

        phi   = atan2(2*(e0*e1 + e2*e3), e0^2 + e3^2 - e1^2 - e2^2);
        theta = asin(2*(e0*e2 - e1*e3));
        psi   = atan2(2*(e0*e3 + e1*e2), e0^2 + e1^2 - e2^2 - e3^2);

        x_euler = [x_quat(1:6); phi; theta; psi; x_quat(11:13)];
    else
        % already Euler representation
        x_euler = x_quat;
    end
end


% =========================================================
% quaternion_state
% convert Euler state to quaternion state
% euler state:      [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r] (12x1)
% quaternion state: [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]  (13x1)
% =========================================================
function x_quat = quaternion_state(x_euler)
    phi   = x_euler(7);
    theta = x_euler(8);
    psi   = x_euler(9);

    e0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
    e1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
    e2 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
    e3 = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);

    x_quat = [x_euler(1:6); e0; e1; e2; e3; x_euler(10:12)];
end


% =========================================================
% f_euler
% evaluates xdot = f(x_euler, delta) using forces_moments
% and mav_dynamics equations
% =========================================================
function xdot = f_euler(x_euler, delta, P)
    wind  = zeros(6,1);
    out   = forces_moments(x_euler, delta, wind, P);
    FM    = out(1:6);

    % call mav_dynamics with flag=1 (derivatives)
    xdot  = mav_dynamics(0, x_euler, FM, 1, P);
end


% =========================================================
% df_dx
% numerical Jacobian of f with respect to state x
% A = df/dx|_(x_trim, u_trim)
% Each column: (f(x + eps*ei, u) - f(x, u)) / eps
% =========================================================
function A = df_dx(x_euler, delta, eps, P)
    n     = length(x_euler);
    f0    = f_euler(x_euler, delta, P);
    A     = zeros(n, n);

    for i = 1:n
        x_plus    = x_euler;
        x_plus(i) = x_plus(i) + eps;
        f_plus    = f_euler(x_plus, delta, P);
        A(:,i)    = (f_plus - f0) / eps;
    end
end


% =========================================================
% df_du
% numerical Jacobian of f with respect to input delta
% B = df/du|_(x_trim, u_trim)
% Each column: (f(x, u + eps*ei) - f(x, u)) / eps
% =========================================================
function B = df_du(x_euler, delta, eps, P)
    nx    = length(x_euler);
    nu    = length(delta);
    f0    = f_euler(x_euler, delta, P);
    B     = zeros(nx, nu);

    for i = 1:nu
        delta_plus    = delta;
        delta_plus(i) = delta_plus(i) + eps;
        f_plus        = f_euler(x_euler, delta_plus, P);
        B(:,i)        = (f_plus - f0) / eps;
    end
end