% find Va range for CL = 0.7 to 1.0 with n = 1.2
P = cessna620_parameters();
n = 1.2;

% from lift = weight * n:  0.5*rho*Va^2*S*CL = mass*g*n
% solve for Va: Va = sqrt(2*mass*g*n / (rho*S*CL))

Va_for_CL07 = sqrt(2*P.mass*P.gravity*n / (P.rho*P.S_wing*0.7));
Va_for_CL10 = sqrt(2*P.mass*P.gravity*n / (P.rho*P.S_wing*1.0));

fprintf('CL=0.7 → Va = %.2f m/s\n', Va_for_CL07);
fprintf('CL=1.0 → Va = %.2f m/s\n', Va_for_CL10);
fprintf('Pick any Va between these two values\n');