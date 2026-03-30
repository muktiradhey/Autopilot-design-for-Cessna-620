function mav_dynamics(block)
% Level-2 MATLAB S-Function for MAV equations of motion
% Beard & McLain, Equations 3.14 - 3.17
%
% SIMULINK S-FUNCTION BLOCK SETTINGS:
%   S-function name:       mav_dynamics
%   S-function parameters: P

setup(block);
end

%% ---- setup ----
function setup(block)
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    block.InputPort(1).Dimensions        = 6;     % [fx fy fz l m n]
    block.InputPort(1).DirectFeedthrough = false;

    block.OutputPort(1).Dimensions = 12;  % all states

    block.NumDialogPrms  = 1;   % just P struct
    block.NumContStates  = 12;
    block.SampleTimes    = [0 0];  % continuous

    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Derivatives',          @Derivatives);
    block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortSamplingMode);
end

%% ---- sampling mode ----
function SetInputPortSamplingMode(block, idx, fd)
    block.InputPort(idx).SamplingMode = fd;
    block.OutputPort(1).SamplingMode  = fd;
end

%% ---- initial conditions ----
% THIS is what was wrong before — Level-2 re-reads P every time
% so assignin('base','P',P) always works correctly
function InitConditions(block)
    P = block.DialogPrm(1).Data;

    % build IC from P — these are set to trim values by run script
    IC = [P.pn0; P.pe0; P.pd0;
          P.u0;  P.v0;  P.w0;
          P.phi0; P.theta0; P.psi0;
          P.p0;  P.q0;  P.r0];

    block.ContStates.Data = IC;

    % print to confirm correct ICs are loaded
    fprintf('[mav_dynamics] theta0 = %.4f deg\n', P.theta0*180/pi);
    fprintf('[mav_dynamics] u0     = %.4f ft/s\n', P.u0);
end

%% ---- outputs ----
function Outputs(block)
    block.OutputPort(1).Data = block.ContStates.Data;
end

%% ---- derivatives ----
function Derivatives(block)
    P = block.DialogPrm(1).Data;

    % unpack inertia
    mass = P.mass;
    Jx   = P.Jx;
    Jy   = P.Jy;
    Jz   = P.Jz;
    Jxz  = P.Jxz;

    % unpack states
    s     = block.ContStates.Data;
    pn    = s(1);  pe    = s(2);  pd  = s(3);
    ub    = s(4);  v     = s(5);  w   = s(6);
    phi   = s(7);  theta = s(8);  psi = s(9);
    p     = s(10); q     = s(11); r   = s(12);

    % unpack inputs [fx fy fz l m n]
    fx = block.InputPort(1).Data(1);
    fy = block.InputPort(1).Data(2);
    fz = block.InputPort(1).Data(3);
    l  = block.InputPort(1).Data(4);
    m  = block.InputPort(1).Data(5);
    n  = block.InputPort(1).Data(6);

    % Gamma coefficients (Eq. 3.13)
    G  = Jx*Jz - Jxz^2;
    G1 = Jxz*(Jx - Jy + Jz) / G;
    G2 = (Jz*(Jz - Jy) + Jxz^2) / G;
    G3 = Jz / G;
    G4 = Jxz / G;
    G5 = (Jz - Jx) / Jy;
    G6 = Jxz / Jy;
    G7 = ((Jx - Jy)*Jx + Jxz^2) / G;
    G8 = Jx / G;

    % trig shorthands
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta); tth = tan(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    % Eq 3.14 - translational kinematics
    pn_dot = cth*cpsi*ub + (sphi*sth*cpsi - cphi*spsi)*v + (cphi*sth*cpsi + sphi*spsi)*w;
    pe_dot = cth*spsi*ub + (sphi*sth*spsi + cphi*cpsi)*v + (cphi*sth*spsi - sphi*cpsi)*w;
    pd_dot = -sth*ub     +  sphi*cth*v                   +  cphi*cth*w;

    % Eq 3.15 - translational dynamics
    u_dot = r*v  - q*w  + fx/mass;
    v_dot = p*w  - r*ub + fy/mass;
    w_dot = q*ub - p*v  + fz/mass;

    % Eq 3.16 - rotational kinematics
    phi_dot   = p + sphi*tth*q + cphi*tth*r;
    theta_dot =     cphi*q     - sphi*r;
    psi_dot   =     sphi/cth*q + cphi/cth*r;

    % Eq 3.17 - rotational dynamics
    p_dot = G1*p*q - G2*q*r + G3*l + G4*n;
    q_dot = G5*p*r - G6*(p^2 - r^2) + m/Jy;
    r_dot = G7*p*q - G1*q*r + G4*l + G8*n;

    % assemble
    block.Derivatives.Data = [pn_dot; pe_dot; pd_dot; ...
                               u_dot;  v_dot;  w_dot;  ...
                               phi_dot; theta_dot; psi_dot; ...
                               p_dot;  q_dot;  r_dot];
end