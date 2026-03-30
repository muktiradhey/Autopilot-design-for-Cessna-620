% get_trim.m — get trim state for given Va, gamma, R
clear; clc;
P = cessna620_parameters();
% pick Va from the range above
Va    = 69.67;    % m/s — adjust until CL is 0.7-1.0
gamma = 0;     % level turn
n     = 1.2;

% compute turn geometry
phi_turn = acos(1/n);
R        = Va^2 / (P.gravity * tan(phi_turn));

fprintf('phi = %.2f deg\n', phi_turn*180/pi);
fprintf('R   = %.2f m\n',   R);

[x_trim, u_trim] = compute_trim(Va, gamma, R, P);

alpha = atan2(x_trim(6), x_trim(4)) * 180/pi;
CL    = P.C_L_0 + P.C_L_alpha * atan2(x_trim(6), x_trim(4));

fprintf('\n===== TRIM STATE x_trim =====\n');
fprintf('  pn    = %.4f m\n',     x_trim(1));
fprintf('  pe    = %.4f m\n',     x_trim(2));
fprintf('  pd    = %.4f m\n',     x_trim(3));
fprintf('  u     = %.4f m/s\n',   x_trim(4));
fprintf('  v     = %.4f m/s\n',   x_trim(5));
fprintf('  w     = %.4f m/s\n',   x_trim(6));
fprintf('  phi   = %.4f deg\n',   x_trim(7)*180/pi);
fprintf('  theta = %.4f deg\n',   x_trim(8)*180/pi);
fprintf('  psi   = %.4f deg\n',   x_trim(9)*180/pi);
fprintf('  p     = %.4f rad/s\n', x_trim(10));
fprintf('  q     = %.4f rad/s\n', x_trim(11));
fprintf('  r     = %.4f rad/s\n', x_trim(12));

fprintf('\n===== TRIM INPUTS u_trim =====\n');
fprintf('  delta_e = %.4f rad (%.2f deg)\n', u_trim(1), u_trim(1)*180/pi);
fprintf('  delta_a = %.4f rad (%.2f deg)\n', u_trim(2), u_trim(2)*180/pi);
fprintf('  delta_r = %.4f rad (%.2f deg)\n', u_trim(3), u_trim(3)*180/pi);
fprintf('  delta_t = %.4f\n',                u_trim(4));

fprintf('\n===== DERIVED VALUES =====\n');
fprintf('  alpha = %.4f deg\n', alpha);
fprintf('  CL    = %.4f\n',     CL);
fprintf('  Va    = %.4f m/s\n', sqrt(x_trim(4)^2+x_trim(5)^2+x_trim(6)^2));