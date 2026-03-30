function forces_moments_demo()
% forces_moments_demo  —  AE700 Part 3  [Cessna 620 parameters]
% Black background, clean non-overlapping layout.
% Units: SI (m, kg, N, m/s, N*m)
% Run: forces_moments_demo

close all;

%% --- Parameters (Cessna 620) --- SI units ---
% Basic constants
ac.g   = 9.81;    % m/s^2
ac.rho = 1.225;   % kg/m^3 (sea-level standard atmosphere)

% Mass / geometry  (converted from US customary)
ac.m    = 15000 * 0.453592;   % kg   (15000 lb)
ac.S    = 340   * 0.092903;   % m^2  (340 ft^2)
ac.b    = 55.1  * 0.3048;     % m    (55.1 ft)
ac.cbar = 6.58  * 0.3048;     % m    (6.58 ft)

% Inertia  (slug*ft^2 x 1.35582 = kg*m^2)
ac.Ixx = 64811 * 1.35582;
ac.Iyy = 17300 * 1.35582;
ac.Izz = 64543 * 1.35582;
ac.Ixz = 0.10 * ac.Ixx;   % 0.1 * Ixx for visible roll-yaw coupling

% ---- Longitudinal aero coefficients ----
ac.CL0   =  0.48;
ac.CLa   =  5.55;
ac.CLq   =  7.5;
ac.CLde  =  0.58;

% Linear drag model: CD = CD0 + CD_alpha * alpha  (Cessna 620 format)
ac.CD0       = 0.0322;
ac.CD_alpha  = 0.269;

ac.Cm0   =  0.06;
ac.Cma   = -1.18;
ac.Cmq   = -22.4;   % pitch-rate damping (stored; used in fm_compute)
ac.Cmde  = -1.73;

% ---- Lateral aero coefficients ----
ac.CYb   = -0.883;
ac.CYp   = -0.227;
ac.CYr   =  0.448;
ac.CYda  =  0;
ac.CYdr  =  0.2;

ac.Clb   = -0.1381;
ac.Clp   = -0.566;
ac.Clr   =  0.1166;
ac.Clda  = -0.1776;
ac.Cldr  =  0.02;

ac.Cnb   =  0.1739;
ac.Cnp   = -0.0501;
ac.Cnr   = -0.2;
ac.Cnda  =  0.0194;
ac.Cndr  = -0.1054;

% Max thrust (N)  (2500 lbf x 4.44822)
ac.T_max = 2500 * 4.44822;

ac = compute_gamma(ac);

% Scale geometry for 3-D animation (scale factor based on wingspan)
sc = ac.b / 4;   % larger scale - aircraft fills more of the viewport
geom = make_geom(sc);

% --- Trim / initial conditions ---
% Approximate level-flight trim at ~61 m/s (~118 kt)
V0      = 200 * 0.3048;                 % m/s  (200 ft/s)
q_trim  = 0.5 * ac.rho * V0^2;
CL_trim = (ac.m * ac.g) / (q_trim * ac.S);
alpha0  = (CL_trim - ac.CL0) / ac.CLa;  % rad
% Trim elevator: Cm = 0 -> delta_e = -(Cm0 + Cma*alpha) / Cmde
delta_e0 = -(ac.Cm0 + ac.Cma * alpha0) / ac.Cmde;
T0       = 0.5 * ac.T_max;              % 50% throttle as starting point
x0 = zeros(12,1);
x0(1) = V0 * cos(alpha0);   % u body-axis (m/s)
x0(3) = V0 * sin(alpha0);   % w body-axis (m/s)
x0(5) = alpha0;              % theta ~= alpha for wings-level trim

HW = 25; dt = 0.02;         % half-window in metres

%% --- Colours ---
BG   = [1.00 1.00 1.00];   % figure background
ABG  = [1.00 1.00 1.00];   % axes background
GRD  = [0.80 0.80 0.80];   % grid lines
LBL  = [0.25 0.25 0.25];   % label colour
VAL  = [0.08 0.18 0.50];   % value colour
CTXT = [0.20 0.20 0.20];   % control text
CSLB = [0.95 0.95 0.95];   % slider label bg

%% --- Figure ---
fig = figure('Name','AE700 Part 3 — Forces, Moments & Aero  [Cessna 620]', ...
             'Position',[40 40 1400 780], 'Color',BG);

setappdata(fig,'running',0);
setappdata(fig,'x_cur',x0);
setappdata(fig,'t_cur',0);
setappdata(fig,'traj_x',0);
setappdata(fig,'traj_y',0);
setappdata(fig,'traj_z',0);
setappdata(fig,'ac',ac);

% --- History storage ---
setappdata(fig,'t_hist',[]);

setappdata(fig,'Fx_hist',[]);
setappdata(fig,'Fy_hist',[]);
setappdata(fig,'Fz_hist',[]);

setappdata(fig,'Mx_hist',[]);
setappdata(fig,'My_hist',[]);
setappdata(fig,'Mz_hist',[]);

setappdata(fig,'phi_hist',[]);
setappdata(fig,'theta_hist',[]);
setappdata(fig,'psi_hist',[]);

setappdata(fig,'Va_hist',[]);
setappdata(fig,'alpha_hist',[]);
setappdata(fig,'beta_hist',[]);

%% -----------------------------------------------------------------------
%  Layout constants (normalised)
%  Figure divided into:
%    LEFT   [0.00 - 0.44]  3-D animation
%    MIDDLE [0.45 - 0.72]  aero readout (top) + controls (bottom)
%    RIGHT  [0.73 - 1.00]  state readout (top) + gust sliders (bottom)
%  TOP    [0.25 - 1.00]   readouts and 3-D axes
%  BOTTOM [0.00 - 0.24]   slider panel
%% -----------------------------------------------------------------------

%% --- 3-D animation axes ---
ax = axes('Parent',fig,'Position',[0.01 0.25 0.43 0.72]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
set(ax,'Color',ABG,'GridColor',GRD,'XColor',LBL,'YColor',LBL,'ZColor',LBL, ...
       'ZDir','reverse','YDir','reverse','FontSize',8,'Box','on');
xlabel(ax,'X_e (m)','Color',LBL);
ylabel(ax,'Y_e (m)','Color',LBL);
zlabel(ax,'Z_e (m)','Color',LBL);
title(ax,'Cessna 620 — Aero + EOM + Animation','FontSize',10,'FontWeight','bold','Color',[0.12 0.25 0.50]);
view(ax,35,25);
xlim(ax,[-HW HW]); ylim(ax,[-HW HW]); zlim(ax,[-HW HW]);
plot3(ax,0,0,0,'r.','MarkerSize',12);

%% -----------------------------------------------------------------------
%  Readout panels — using uicontrol text rows for clean alignment
%  Each row: [label uicontrol] + [value uicontrol]
%  Columns are spaced manually in normalised coords
%% -----------------------------------------------------------------------

% --- Aero readout ---
% 15 values arranged in 3 columns of 5 rows
% Available x-space: 0.455 to 0.840  (width 0.385)
% Each column gets 0.385/3 = 0.128; label width 0.085, value width 0.040

aero_names = {'Va (m/s)','alpha(deg)','beta (deg)','wn (m/s)','we (m/s)', ...
              'wd (m/s)','CL','CD','Cm','Fx (N)', ...
              'Fy (N)','Fz (N)','l (N*m)','m (N*m)','n (N*m)'};

aero_txt = cell(15,1);

% 3 columns, 5 rows — tighter packing with wider label boxes
col_lx = [0.455  0.588  0.721];   % label left edges
col_vx = [0.543  0.676  0.809];   % value left edges (label_lx + 0.088)
lw     = 0.085;                    % label width
vw     = 0.038;                    % value width
row_y  = fliplr(linspace(0.58, 0.93, 5));  % 5 rows top-to-bottom
rh     = 0.048;  % row height

% Header
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.455 0.955 0.385 0.032], ...
    'String','Aero outputs', ...
    'BackgroundColor',BG,'ForegroundColor',[0.20 0.20 0.20], ...
    'FontSize',9,'FontWeight','bold','HorizontalAlignment','left');

for k = 1:15
    col = floor((k-1)/5);
    row = mod(k-1,5);
    lx = col_lx(col+1);
    vx = col_vx(col+1);
    ry = row_y(row+1);

    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[lx ry lw rh], ...
        'String',aero_names{k}, ...
        'BackgroundColor',BG,'ForegroundColor',LBL, ...
        'FontSize',8,'HorizontalAlignment','left');

    aero_txt{k} = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[vx ry vw rh], ...
        'String','0.00', ...
        'BackgroundColor',BG,'ForegroundColor',VAL, ...
        'FontSize',8,'FontWeight','bold','HorizontalAlignment','right');
end

% --- State readout ---
% 12 values in 2 columns of 6 rows
% Available x-space: 0.845 to 0.995  (width 0.150)
% Each column gets 0.075; label width 0.048, value width 0.026
state_names = {'u (m/s)','v (m/s)','w (m/s)','phi(deg)','the(deg)','psi(deg)', ...
               'xe (m)','ye (m)','ze (m)','p(d/s)','q(d/s)','r(d/s)'};
state_txt = cell(12,1);

scol_lx = [0.845  0.920];
scol_vx = [0.893  0.968];
slw     = 0.046;
svw     = 0.025;
row_sy  = fliplr(linspace(0.58, 0.93, 6));

uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.845 0.955 0.150 0.032], ...
    'String','State', ...
    'BackgroundColor',BG,'ForegroundColor',[0.7 0.7 0.7], ...
    'FontSize',9,'FontWeight','bold','HorizontalAlignment','left');

for k = 1:12
    col = floor((k-1)/6);
    row = mod(k-1,6);
    lx  = scol_lx(col+1);
    vx  = scol_vx(col+1);
    ry  = row_sy(row+1);

    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[lx ry slw rh], ...
        'String',state_names{k}, ...
        'BackgroundColor',BG,'ForegroundColor',LBL, ...
        'FontSize',8,'HorizontalAlignment','left');

    state_txt{k} = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[vx ry svw rh], ...
        'String','0.00', ...
        'BackgroundColor',BG,'ForegroundColor',VAL, ...
        'FontSize',8,'FontWeight','bold','HorizontalAlignment','right');
end

%% -----------------------------------------------------------------------
%  Bottom panel: control surface sliders (left) + gust sliders (right)
%  4 rows: de, da, dr, throttle  |  3 rows: ug, vg, wg
%  Bottom panel occupies y = 0.01 to 0.23
%% -----------------------------------------------------------------------

% Divider line annotation
annotation(fig,'line',[0.445 0.445],[0.01 0.23],'Color',GRD,'LineWidth',0.5);
annotation(fig,'line',[0.445 1.000],[0.235 0.235],'Color',GRD,'LineWidth',0.5);

% Section headers
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.01 0.195 0.20 0.03],'String','Control surfaces', ...
    'BackgroundColor',BG,'ForegroundColor',[0.20 0.20 0.20], ...
    'FontSize',8,'FontWeight','bold','HorizontalAlignment','left');

uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.455 0.195 0.15 0.03],'String','Wind gusts', ...
    'BackgroundColor',BG,'ForegroundColor',[0.65 0.65 0.65], ...
    'FontSize',8,'FontWeight','bold','HorizontalAlignment','left');

% Control surface sliders
cs_names = {'delta\_e (deg)','delta\_a (deg)','delta\_r (deg)','Throttle (0-1)'};
cs_mins  = [-30 -30 -30  0];
cs_maxs  = [ 30  30  30  1];
cs_vals  = [rad2deg(delta_e0)  0  0  T0/ac.T_max];
cs_steps = {[1/60 5/60],[1/60 5/60],[1/60 5/60],[0.01 0.1]};

cs_sl  = cell(4,1);
cs_lbl = cell(4,1);

for k = 1:4
    ry = 0.165 - (k-1)*0.048;
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.01 ry 0.10 0.036], ...
        'String',cs_names{k}, ...
        'BackgroundColor',BG,'ForegroundColor',CTXT, ...
        'FontSize',8,'HorizontalAlignment','left');
    cs_sl{k} = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[0.12 ry 0.25 0.036], ...
        'Min',cs_mins(k),'Max',cs_maxs(k),'Value',cs_vals(k), ...
        'SliderStep',[0.05 0.20], ...
        'BackgroundColor',[0.88 0.88 0.88]);
    cs_lbl{k} = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.375 ry 0.062 0.036], ...
        'String',sprintf('%.2f',cs_vals(k)), ...
        'BackgroundColor',BG,'ForegroundColor',VAL, ...
        'FontSize',8,'FontWeight','bold','HorizontalAlignment','right');
end
for k=1:4
    cs_sl{k}.Callback = @(s,e) update_labels(cs_sl,cs_lbl,'%.2f');
end

% Gust sliders
g_names = {'ug (m/s)','vg (m/s)','wg (m/s)'};
g_sl  = cell(3,1);
g_lbl = cell(3,1);
for k = 1:3
    ry = 0.165 - (k-1)*0.048;
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.455 ry 0.072 0.036], ...
        'String',g_names{k}, ...
        'BackgroundColor',BG,'ForegroundColor',CTXT, ...
        'FontSize',8,'HorizontalAlignment','left');
    g_sl{k} = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[0.530 ry 0.22 0.036], ...
        'Min',-20,'Max',20,'Value',0,'SliderStep',[0.05 0.20], ...
        'BackgroundColor',[0.88 0.88 0.88]);
    g_lbl{k} = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.755 ry 0.052 0.036], ...
        'String','0.0', ...
        'BackgroundColor',BG,'ForegroundColor',VAL, ...
        'FontSize',8,'FontWeight','bold','HorizontalAlignment','right');
end
for k=1:3
    g_sl{k}.Callback = @(s,e) update_labels(g_sl,g_lbl,'%.1f');
end

%% --- Buttons, time, checkboxes ---
grav_chk = uicontrol(fig,'Style','checkbox','Units','normalized', ...
    'Position',[0.455 0.025 0.15 0.035], ...
    'String','Disable gravity', ...
    'BackgroundColor',BG,'ForegroundColor',CTXT,'FontSize',8,'Value',0);

time_txt = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.01 0.01 0.16 0.035], ...
    'String','t = 0.00 s', ...
    'BackgroundColor',[0.94 0.96 0.99],'ForegroundColor',[0.08 0.18 0.45], ...
    'FontSize',10,'FontWeight','bold','HorizontalAlignment','left');

btn_run = uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.815 0.055 0.085 0.048], ...
    'String','Run','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.25 0.65 0.35],'ForegroundColor','white');

btn_reset = uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.910 0.055 0.080 0.048], ...
    'String','Reset','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.90 0.55 0.45],'ForegroundColor','white');

%% --- Initial draw ---
hac   = make_aircraft(ax, eye(3), [0;0;0], geom);
htraj = plot3(ax,0,0,0,'-','Color',[0.2 0.6 1.0],'LineWidth',0.8);

%% --- Button callbacks ---
btn_run.Callback = @(s,e) toggle_run(fig, btn_run, cs_sl, g_sl, grav_chk, ...
                                      hac, htraj, ax, state_txt, aero_txt, ...
                                      time_txt, geom, dt, HW);

btn_reset.Callback = @(s,e) do_reset(fig, btn_run, cs_sl, cs_lbl, cs_vals, ...
                                      g_sl, g_lbl, grav_chk, hac, htraj, ax, ...
                                      state_txt, aero_txt, time_txt, geom, x0, ac, HW);
end

%% =========================================================================
function update_labels(sl, lbl, fmt)
    for k = 1:numel(sl)
        lbl{k}.String = sprintf(fmt, sl{k}.Value);
    end
end

%% =========================================================================
function toggle_run(fig, btn_run, cs_sl, g_sl, grav_chk, ...
                    hac, htraj, ax, state_txt, aero_txt, time_txt, geom, dt, HW)
    if ~ishandle(fig), return; end
    if getappdata(fig,'running')
        setappdata(fig,'running',0);
        btn_run.String='Run';
        btn_run.BackgroundColor=[0.25 0.65 0.35];
        btn_run.ForegroundColor='white';
        return;
    end
    setappdata(fig,'running',1);
    btn_run.String='Stop';
    btn_run.BackgroundColor=[0.80 0.20 0.20];
    btn_run.ForegroundColor='white';

    while ishandle(fig) && getappdata(fig,'running')
        step_clock = tic;
        ac       = getappdata(fig,'ac');
        x_cur    = getappdata(fig,'x_cur');
        t_cur    = getappdata(fig,'t_cur');
        use_grav = ~grav_chk.Value;

        de  = deg2rad(cs_sl{1}.Value);
        da  = deg2rad(cs_sl{2}.Value);
        dr  = deg2rad(cs_sl{3}.Value);
        thr = cs_sl{4}.Value;
        gust = [g_sl{1}.Value; g_sl{2}.Value; g_sl{3}.Value];

        [F, M, aout] = fm_compute(x_cur, de, da, dr, thr, gust, ac, use_grav);
        u_in = [F; M];

        [~,xout] = ode45(@(t,x) eom(t,x,u_in,ac), [t_cur t_cur+dt], x_cur);
        x_cur = xout(end,:)';
        t_cur = t_cur + dt;
        x_cur(4)=wrapToPi(x_cur(4)); x_cur(5)=wrapToPi(x_cur(5)); x_cur(6)=wrapToPi(x_cur(6));

        setappdata(fig,'x_cur',x_cur); setappdata(fig,'t_cur',t_cur);

        pos=x_cur(7:9); R=make_dcm(x_cur(4),x_cur(5),x_cur(6));
        update_aircraft(hac,R,pos,geom);

        tx=getappdata(fig,'traj_x'); ty=getappdata(fig,'traj_y'); tz=getappdata(fig,'traj_z');
        tx(end+1)=pos(1); ty(end+1)=pos(2); tz(end+1)=pos(3);

        % --- Store history ---
t_hist = getappdata(fig,'t_hist');

Fx_hist = getappdata(fig,'Fx_hist');
Fy_hist = getappdata(fig,'Fy_hist');
Fz_hist = getappdata(fig,'Fz_hist');

Mx_hist = getappdata(fig,'Mx_hist');
My_hist = getappdata(fig,'My_hist');
Mz_hist = getappdata(fig,'Mz_hist');

phi_hist   = getappdata(fig,'phi_hist');
theta_hist = getappdata(fig,'theta_hist');
psi_hist   = getappdata(fig,'psi_hist');

Va_hist    = getappdata(fig,'Va_hist');
alpha_hist = getappdata(fig,'alpha_hist');
beta_hist  = getappdata(fig,'beta_hist');

% append
t_hist(end+1) = t_cur;

Fx_hist(end+1) = F(1);
Fy_hist(end+1) = F(2);
Fz_hist(end+1) = F(3);

Mx_hist(end+1) = M(1);
My_hist(end+1) = M(2);
Mz_hist(end+1) = M(3);

phi_hist(end+1)   = rad2deg(x_cur(4));
theta_hist(end+1) = rad2deg(x_cur(5));
psi_hist(end+1)   = rad2deg(x_cur(6));

Va_hist(end+1)    = aout.Va;
alpha_hist(end+1) = rad2deg(aout.alpha);
beta_hist(end+1)  = rad2deg(aout.beta);

% save back
setappdata(fig,'t_hist',t_hist);

setappdata(fig,'Fx_hist',Fx_hist);
setappdata(fig,'Fy_hist',Fy_hist);
setappdata(fig,'Fz_hist',Fz_hist);

setappdata(fig,'Mx_hist',Mx_hist);
setappdata(fig,'My_hist',My_hist);
setappdata(fig,'Mz_hist',Mz_hist);

setappdata(fig,'phi_hist',phi_hist);
setappdata(fig,'theta_hist',theta_hist);
setappdata(fig,'psi_hist',psi_hist);

setappdata(fig,'Va_hist',Va_hist);
setappdata(fig,'alpha_hist',alpha_hist);
setappdata(fig,'beta_hist',beta_hist);

        setappdata(fig,'traj_x',tx); setappdata(fig,'traj_y',ty); setappdata(fig,'traj_z',tz);
        set(htraj,'XData',tx,'YData',ty,'ZData',tz);

        set(ax,'XLim',pos(1)+[-HW HW],'YLim',pos(2)+[-HW HW],'ZLim',pos(3)+[-HW HW]);
        update_state_display(state_txt, x_cur);
        update_aero_display(aero_txt, aout, F, M);
        time_txt.String = sprintf('t = %.2f s', t_cur);
        drawnow;
        elapsed=toc(step_clock);
        if dt-elapsed>0, pause(dt-elapsed); end
    end
% --- Generate plots when simulation stops ---
plot_results(fig);

    if ishandle(fig)
        btn_run.String='Run';
        btn_run.BackgroundColor=[0.25 0.65 0.35];
        btn_run.ForegroundColor='white';
        setappdata(fig,'running',0);
    end
end

%% =========================================================================
function do_reset(fig, btn_run, cs_sl, cs_lbl, cs_vals, g_sl, g_lbl, ...
                  grav_chk, hac, htraj, ax, state_txt, aero_txt, time_txt, ...
                  geom, x0, ac, HW)
    if ~ishandle(fig), return; end
    setappdata(fig,'running',0); drawnow; pause(0.05);
    btn_run.String='Run'; btn_run.BackgroundColor=[0.25 0.65 0.35]; btn_run.ForegroundColor='white';
    setappdata(fig,'x_cur',x0); setappdata(fig,'t_cur',0);
    setappdata(fig,'traj_x',0); setappdata(fig,'traj_y',0); setappdata(fig,'traj_z',0);
    setappdata(fig,'ac',ac);
    for k=1:4
        set(cs_sl{k}, 'Value', cs_vals(k));
        cs_lbl{k}.String = sprintf('%.2f', cs_vals(k));
    end
    drawnow;
    for k=1:3
        set(g_sl{k}, 'Value', 0);
        g_lbl{k}.String = '0.0';
    end
    drawnow;
    grav_chk.Value=0;
    set(htraj,'XData',0,'YData',0,'ZData',0);
    time_txt.String='t = 0.00 s';
    update_aircraft(hac,eye(3),x0(7:9),geom);
    set(ax,'XLim',[-HW HW],'YLim',[-HW HW],'ZLim',[-HW HW]);
    update_state_display(state_txt,x0);
    for k=1:15, aero_txt{k}.String='0.00'; end
    drawnow;
end

%% =========================================================================
function [F,M,aout] = fm_compute(x,de,da,dr,thr,gust,ac,use_grav)
    u=x(1);v=x(2);w=x(3); phi=x(4);theta=x(5); p=x(10);q=x(11);r=x(12);
    ur=u-gust(1); vr=v-gust(2); wr=w-gust(3);
    Va=max(sqrt(ur^2+vr^2+wr^2),1);
    alpha=atan2(wr,ur); beta=asin(vr/Va);
    R=make_dcm(phi,theta,x(6)); w_ned=R*gust;
    qbar=0.5*ac.rho*Va^2;
    phat=p*ac.b/(2*Va); qhat=q*ac.cbar/(2*Va); rhat=r*ac.b/(2*Va);
    CL=ac.CL0+ac.CLa*alpha+ac.CLq*qhat+ac.CLde*de;
    CD=max(ac.CD0 + ac.CD_alpha*abs(alpha), 0);
    CY=ac.CYb*beta+ac.CYp*phat+ac.CYr*rhat+ac.CYda*da+ac.CYdr*dr;
    Cl=ac.Clb*beta+ac.Clp*phat+ac.Clr*rhat+ac.Clda*da+ac.Cldr*dr;
    Cm=ac.Cm0+ac.Cma*alpha+ac.Cmq*qhat+ac.Cmde*de;
    Cn=ac.Cnb*beta+ac.Cnp*phat+ac.Cnr*rhat+ac.Cnda*da+ac.Cndr*dr;
    Fx_a=qbar*ac.S*(-CD*cos(alpha)+CL*sin(alpha))+thr*ac.T_max;
    Fy_a=qbar*ac.S*CY;
    Fz_a=qbar*ac.S*(-CD*sin(alpha)-CL*cos(alpha));
    if use_grav
        Fgx=-ac.m*ac.g*sin(theta); Fgy=ac.m*ac.g*cos(theta)*sin(phi); Fgz=ac.m*ac.g*cos(theta)*cos(phi);
    else
        Fgx=0; Fgy=0; Fgz=0;
    end
    F=[Fx_a+Fgx; Fy_a+Fgy; Fz_a+Fgz];
    M=[qbar*ac.S*ac.b*Cl; qbar*ac.S*ac.cbar*Cm; qbar*ac.S*ac.b*Cn];
    aout.Va=Va; aout.alpha=alpha; aout.beta=beta; aout.wned=w_ned;
    aout.CL=CL; aout.CD=CD; aout.Cm=Cm;
end

%% =========================================================================
function xdot=eom(~,x,u,ac)
    ub=x(1);vb=x(2);wb=x(3); phi=x(4);theta=x(5);psi=x(6);
    pr=x(10);qr=x(11);rr=x(12);
    Fx=u(1);Fy=u(2);Fz=u(3); l=u(4);mm=u(5);n=u(6);
    udot=rr*vb-qr*wb+Fx/ac.m; vdot=pr*wb-rr*ub+Fy/ac.m; wdot=qr*ub-pr*vb+Fz/ac.m;
    pdot=ac.G1*pr*qr-ac.G2*qr*rr+ac.G3*l+ac.G4*n;
    qdot=ac.G5*pr*rr-ac.G6*(pr^2-rr^2)+mm/ac.Iyy;
    rdot=ac.G7*pr*qr-ac.G1*qr*rr+ac.G4*l+ac.G8*n;
    phidot=pr+(qr*sin(phi)+rr*cos(phi))*tan(theta);
    thetadot=qr*cos(phi)-rr*sin(phi);
    psidot=(qr*sin(phi)+rr*cos(phi))/cos(theta);
    cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);cs=cos(psi);ss=sin(psi);
    xedot=(ct*cs)*ub+(sp*st*cs-cp*ss)*vb+(cp*st*cs+sp*ss)*wb;
    yedot=(ct*ss)*ub+(sp*st*ss+cp*cs)*vb+(cp*st*ss-sp*cs)*wb;
    zedot=(-st)*ub+(sp*ct)*vb+(cp*ct)*wb;
    xdot=[udot;vdot;wdot;phidot;thetadot;psidot;xedot;yedot;zedot;pdot;qdot;rdot];
end

%% =========================================================================
function ac=compute_gamma(ac)
    G=ac.Ixx*ac.Izz-ac.Ixz^2;
    ac.G1=ac.Ixz*(ac.Ixx-ac.Iyy+ac.Izz)/G; ac.G2=(ac.Izz*(ac.Izz-ac.Iyy)+ac.Ixz^2)/G;
    ac.G3=ac.Izz/G; ac.G4=ac.Ixz/G; ac.G5=(ac.Izz-ac.Ixx)/ac.Iyy;
    ac.G6=ac.Ixz/ac.Iyy; ac.G7=(ac.Ixx*(ac.Ixx-ac.Iyy)+ac.Ixz^2)/G; ac.G8=ac.Ixx/G;
end

function update_state_display(state_txt,x)
    vals=[x(1:3);rad2deg(x(4:6));x(7:9);rad2deg(x(10:12))];
    for k=1:12, state_txt{k}.String=sprintf('%.2f',vals(k)); end
end

function update_aero_display(aero_txt,aout,F,M)
    vals={aout.Va,rad2deg(aout.alpha),rad2deg(aout.beta), ...
          aout.wned(1),aout.wned(2),aout.wned(3), ...
          aout.CL,aout.CD,aout.Cm, F(1),F(2),F(3),M(1),M(2),M(3)};
    for k=1:15, aero_txt{k}.String=sprintf('%.2f',vals{k}); end
end

%% =========================================================================
function geom=make_geom(sc)
    geom.fus  =sc*[-4 0 0;4 0 0];
    geom.wing =sc*[0.5 6 0;-1.5 6 0;-1.5 -6 0;0.5 -6 0];
    geom.htail=sc*[-3.5 2 0;-2.5 2 0;-2.5 -2 0;-3.5 -2 0];
    geom.vtail=sc*[-3.5 0 0;-2.5 0 0;-2.5 0 -1.8;-3.5 0 -1.4];
    th=linspace(0,2*pi,20)';
    geom.body=sc*[4*cos(th),zeros(20,1),0.6*sin(th)];
end

function h=make_aircraft(ax,R,t,geom)
    cb=[0.20 0.35 0.65]; cw=[0.25 0.55 0.75]; ct=[0.40 0.45 0.65]; cl=[0.55 0.70 0.90];
    fw=xf(geom.wing,R,t); fht=xf(geom.htail,R,t); fvt=xf(geom.vtail,R,t);
    fbod=xf(geom.body,R,t); ffus=xf(geom.fus,R,t);
    h.wing =fill3(ax,fw(:,1),fw(:,2),fw(:,3),cw,'FaceAlpha',0.8,'EdgeColor',cl,'LineWidth',0.8);
    h.htail=fill3(ax,fht(:,1),fht(:,2),fht(:,3),ct,'FaceAlpha',0.8,'EdgeColor',cl,'LineWidth',0.8);
    h.vtail=fill3(ax,fvt(:,1),fvt(:,2),fvt(:,3),ct,'FaceAlpha',0.8,'EdgeColor',cl,'LineWidth',0.8);
    h.body =fill3(ax,fbod(:,1),fbod(:,2),fbod(:,3),cb,'FaceAlpha',0.6,'EdgeColor',cl,'LineWidth',0.4);
    h.fus  =plot3(ax,ffus(:,1),ffus(:,2),ffus(:,3),'-','Color',cl,'LineWidth',2.5);
    cg=xf([0 0 0],R,t); aX=xf([3 0 0],R,t); aY=xf([0 3 0],R,t); aZ=xf([0 0 3],R,t);
    h.ax_x=plot3(ax,[cg(1) aX(1)],[cg(2) aX(2)],[cg(3) aX(3)],'r-','LineWidth',2);
    h.ax_y=plot3(ax,[cg(1) aY(1)],[cg(2) aY(2)],[cg(3) aY(3)],'g-','LineWidth',2);
    h.ax_z=plot3(ax,[cg(1) aZ(1)],[cg(2) aZ(2)],[cg(3) aZ(3)],'b-','LineWidth',2);
    h.lx=text(ax,aX(1),aX(2),aX(3),' x_b','Color','r','FontSize',8,'FontWeight','bold');
    h.ly=text(ax,aY(1),aY(2),aY(3),' y_b','Color','g','FontSize',8,'FontWeight','bold');
    h.lz=text(ax,aZ(1),aZ(2),aZ(3),' z_b','Color','b','FontSize',8,'FontWeight','bold');
end

function update_aircraft(h,R,t,geom)
    fw=xf(geom.wing,R,t); fht=xf(geom.htail,R,t); fvt=xf(geom.vtail,R,t);
    fbod=xf(geom.body,R,t); ffus=xf(geom.fus,R,t);
    set(h.wing,'XData',fw(:,1),'YData',fw(:,2),'ZData',fw(:,3));
    set(h.htail,'XData',fht(:,1),'YData',fht(:,2),'ZData',fht(:,3));
    set(h.vtail,'XData',fvt(:,1),'YData',fvt(:,2),'ZData',fvt(:,3));
    set(h.body,'XData',fbod(:,1),'YData',fbod(:,2),'ZData',fbod(:,3));
    set(h.fus,'XData',ffus(:,1),'YData',ffus(:,2),'ZData',ffus(:,3));
    cg=xf([0 0 0],R,t); aX=xf([3 0 0],R,t); aY=xf([0 3 0],R,t); aZ=xf([0 0 3],R,t);
    set(h.ax_x,'XData',[cg(1) aX(1)],'YData',[cg(2) aX(2)],'ZData',[cg(3) aX(3)]);
    set(h.ax_y,'XData',[cg(1) aY(1)],'YData',[cg(2) aY(2)],'ZData',[cg(3) aY(3)]);
    set(h.ax_z,'XData',[cg(1) aZ(1)],'YData',[cg(2) aZ(2)],'ZData',[cg(3) aZ(3)]);
    set(h.lx,'Position',aX); set(h.ly,'Position',aY); set(h.lz,'Position',aZ);
end

function pts_w=xf(pts,R,t), pts_w=(R*pts'+t)'; end

function R=make_dcm(phi,theta,psi)
    Rx=[1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
    Ry=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
    Rz=[cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];
    R=Rz*Ry*Rx;

    
end

function plot_results(fig)

t  = getappdata(fig,'t_hist');

Fx = getappdata(fig,'Fx_hist');
Fy = getappdata(fig,'Fy_hist');
Fz = getappdata(fig,'Fz_hist');

Mx = getappdata(fig,'Mx_hist');
My = getappdata(fig,'My_hist');
Mz = getappdata(fig,'Mz_hist');

phi   = getappdata(fig,'phi_hist');
theta = getappdata(fig,'theta_hist');
psi   = getappdata(fig,'psi_hist');

Va    = getappdata(fig,'Va_hist');
alpha = getappdata(fig,'alpha_hist');
beta  = getappdata(fig,'beta_hist');

%% --- Forces & Moments ---
figure('Name','Forces and Moments')

subplot(2,1,1)
plot(t,Fx,t,Fy,t,Fz,'LineWidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Force (N)')
legend('Fx','Fy','Fz')
title('Forces vs Time')

subplot(2,1,2)
plot(t,Mx,t,My,t,Mz,'LineWidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Moment (N*m)')
legend('l (roll)','m (pitch)','n (yaw)')
title('Moments vs Time')

%% --- States & Aero ---
figure('Name','States and Aerodynamics')

subplot(3,2,1)
plot(t,theta,'LineWidth',1.5)
grid on
title('\theta (Pitch Angle)')

subplot(3,2,2)
plot(t,phi,'LineWidth',1.5)
grid on
title('\phi (Roll Angle)')

subplot(3,2,3)
plot(t,psi,'LineWidth',1.5)
grid on
title('\psi (Yaw Angle)')

subplot(3,2,4)
plot(t,Va,'LineWidth',1.5)
grid on
title('Airspeed Va')

subplot(3,2,5)
plot(t,alpha,'LineWidth',1.5)
grid on
title('\alpha (deg)')

subplot(3,2,6)
plot(t,beta,'LineWidth',1.5)
grid on
title('\beta (deg)')

end