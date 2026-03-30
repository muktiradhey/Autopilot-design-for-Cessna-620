function [x_trim, u_trim] = compute_trim(Va, gamma, R, P)
% COMPUTE_TRIM  Find trim state and inputs using Simulink trim command
%
%   [x_trim, u_trim] = compute_trim(Va, gamma, R, P)
%
%   INPUTS
%     Va    - desired trim airspeed  (ft/s)
%     gamma - desired flight-path angle (rad)  [+ = climb]
%     R     - desired turn radius (ft)  [Inf = straight flight]
%     P     - aircraft parameter struct from cessna620_parameters()
%
%   OUTPUTS
%     x_trim - [12x1] trim state  [pn pe pd u v w phi theta psi p q r]'
%     u_trim - [4x1]  trim input  [delta_e delta_a delta_r delta_t]'

    % ---- initial state guess (Appendix F.1) ----
    x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
    ix = [];

    % ---- initial input guess ----
    u0 = [0; 0; 0; 1];   % [delta_e; delta_a; delta_r; delta_t]
    iu = [];

    % ---- desired outputs ----
    y0 = [Va; 0; 0];     % [Va; alpha; beta]
    iy = [1, 3];          % fix Va and beta, leave alpha free

    % ---- desired state derivatives (Eq. 5.21) ----
    if R == Inf
        dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; 0; 0; 0; 0];
    else
        dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va*cos(gamma)/R; 0; 0; 0];
    end
    idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

    % ---- push P to base workspace so trim() can see it ----
    assignin('base', 'P', P);

    % ---- call Simulink trim ----
    [x_trim, u_trim, y_trim, dx_trim] = trim('mavsim_trim', ...
                                              x0, u0, y0, ...
                                              ix, iu, iy, ...
                                              dx0, idx);

    % ---- verify trim worked ----
    residual = norm(dx_trim(3:end) - dx0(3:end));
    fprintf('Trim residual = %.2e  (should be ~0)\n', residual);
    if residual > 1e-3
        warning('Trim residual is large - check model and parameters');
    end

    % ---- print results ----
    alpha_trim = atan2(x_trim(6), x_trim(4)) * 180/pi;
    fprintf('\n========== TRIM RESULTS ==========\n');
    fprintf('Va    = %.2f ft/s\n', Va);
    fprintf('gamma = %.4f rad (%.2f deg)\n', gamma, gamma*180/pi);
    if R == Inf
        fprintf('R     = Inf (straight flight)\n');
    else
        fprintf('R     = %.2f ft\n', R);
    end
    fprintf('----------------------------------\n');
    fprintf('alpha*   = %.4f deg\n', alpha_trim);
    fprintf('theta*   = %.4f deg\n', x_trim(8)*180/pi);
    fprintf('phi*     = %.4f deg\n', x_trim(7)*180/pi);
    fprintf('delta_e* = %.4f rad\n', u_trim(1));
    fprintf('delta_a* = %.4f rad\n', u_trim(2));
    fprintf('delta_r* = %.4f rad\n', u_trim(3));
    fprintf('delta_t* = %.4f\n',     u_trim(4));
    fprintf('==================================\n\n');

    % NOTE: P is NOT updated here intentionally
    % P must be updated in the calling script (run_simulation.m)
    % because MATLAB passes structs by value not by reference
end