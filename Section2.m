function eom_animation_demo()
% AE700 Section 2

close all;

%% Parameters (Cessna 620, imperial) 
P = cessna620_parameters();
p0.m   = P.mass;
p0.g   = P.gravity;
p0.Ixx = P.Jx;
p0.Iyy = P.Jy;
p0.Izz = P.Jz;
p0.Ixz = 0;          % start decoupled
p0     = compute_gamma(p0);

sc   = 0.08;          % visual scale for aircraft geometry
geom = make_geom(sc);

x0   = zeros(12,1);   % everything at rest at origin
HW   = 1.5;
dt   = 0.02;

%%  Figure
fig = figure('Name','AE700 Section 2 — 6DOF EOM', ...
             'Position',[60 60 1200 660], 'Color','white');

setappdata(fig,'running',0);
setappdata(fig,'x_cur',x0);
setappdata(fig,'t_cur',0);
setappdata(fig,'traj_x',0);
setappdata(fig,'traj_y',0);
setappdata(fig,'traj_z',0);
setappdata(fig,'p',p0);

%%  3D axes 
ax = axes('Parent',fig,'Position',[0.01 0.18 0.54 0.78]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
ax.Color     = 'white';
ax.GridColor = [0.80 0.80 0.80];
ax.GridAlpha = 0.6;
ax.XColor    = [0.25 0.25 0.25];
ax.YColor    = [0.25 0.25 0.25];
ax.ZColor    = [0.25 0.25 0.25];
ax.FontSize  = 9;
ax.Box       = 'on';
xlabel(ax,'X_e  (North, ft)','FontSize',9);
ylabel(ax,'Y_e  (East,  ft)','FontSize',9);
zlabel(ax,'Z_e  (Down,  ft)','FontSize',9);
title(ax,'Cessna 620 — 6DOF EOM', ...
    'FontSize',10,'FontWeight','bold','Color',[0.12 0.25 0.50]);
view(ax,35,25);
set(ax,'ZDir','reverse','YDir','reverse');
xlim(ax,[-HW HW]); ylim(ax,[-HW HW]); zlim(ax,[-HW HW]);
plot3(ax,0,0,0,'r.','MarkerSize',14);
text(ax,0.15,0.15,0,'Origin','FontSize',8,'Color',[0.7 0.1 0.1]);

%% State readout panel 
ax_st = axes('Parent',fig,'Position',[0.57 0.48 0.41 0.48]);
axis(ax_st,'off');
text(ax_st,0,1.05,'State  readout', ...
    'FontSize',9,'FontWeight','bold','Color',[0.20 0.20 0.20], ...
    'Units','normalized');

snames = {'u  (ft/s)','v  (ft/s)','w  (ft/s)', ...
          '\phi  (deg)','\theta  (deg)','\psi  (deg)', ...
          'x_e  (ft)','y_e  (ft)','z_e  (ft)', ...
          'p  (deg/s)','q  (deg/s)','r  (deg/s)'};

% Subtle row shading
row_cols = {[0.95 0.97 1.00],[1.00 1.00 1.00]};
state_txt = cell(12,1);
for k = 1:12
    col = floor((k-1)/6);
    row = mod(k-1,6);
    xpos = col*0.52;
    ypos = 1 - row*0.155;
    text(ax_st, xpos, ypos, snames{k}, ...
        'FontSize',8,'Color',[0.45 0.45 0.45],'Units','normalized');
    state_txt{k} = text(ax_st, xpos+0.29, ypos, '0.00', ...
        'FontSize',9,'FontWeight','bold','Units','normalized', ...
        'Color',[0.12 0.25 0.55]);
end

%%  Force/moment sliders 
ctrl_names = {'F_x  (lb)','F_y  (lb)','F_z  (lb)', ...
              '\ell  (ft·lb)','m  (ft·lb)','n  (ft·lb)'};
ctrl_mins  = [-5000 -2000 -5000 -50000 -50000 -50000];
ctrl_maxs  = [ 5000  2000  5000  50000  50000  50000];

% Row colours: forces warm, moments cool
row_bg = {[0.98 0.95 0.92],[0.98 0.95 0.92],[0.98 0.95 0.92], ...
          [0.93 0.95 0.99],[0.93 0.95 0.99],[0.93 0.95 0.99]};

sliders = cell(6,1);
slbls   = cell(6,1);

for k = 1:6
    yp = 0.530 - (k-1)*0.062;

    % Row tint panel
    uipanel(fig,'Units','normalized', ...
        'Position',[0.565 yp-0.005 0.430 0.058], ...
        'BackgroundColor',row_bg{k},'BorderType','none');

    % Name
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.572 yp+0.005 0.095 0.042], ...
        'String',ctrl_names{k}, ...
        'BackgroundColor',row_bg{k}, ...
        'HorizontalAlignment','left','FontSize',8,'FontWeight','bold', ...
        'ForegroundColor',[0.20 0.20 0.20]);

    % Slider
    sliders{k} = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[0.668 yp+0.010 0.290 0.036], ...
        'Min',ctrl_mins(k),'Max',ctrl_maxs(k),'Value',0, ...
        'SliderStep',[0.05, 0.20], ...
        'BackgroundColor',[0.88 0.88 0.88]);

    % Value label
    slbls{k} = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.963 yp+0.005 0.030 0.042], ...
        'String','0', ...
        'BackgroundColor',row_bg{k}, ...
        'FontSize',9,'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'ForegroundColor',[0.10 0.10 0.50]);
end

for k = 1:6
    sliders{k}.Callback = @(s,e) update_slider_labels(sliders,slbls);
end

%%  Jxz coupling checkbox
jxz_chk = uicontrol(fig,'Style','checkbox','Units','normalized', ...
    'Position',[0.572 0.090 0.38 0.048], ...
    'String','Enable J_{xz} roll-yaw coupling  (J_{xz} = 0.1 J_x)', ...
    'BackgroundColor','white','FontSize',9,'Value',0, ...
    'ForegroundColor',[0.20 0.20 0.20]);
jxz_chk.Callback = @(s,e) update_p_appdata(fig,s.Value,p0,P);

%%  Time display 
time_txt = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.572 0.148 0.22 0.048], ...
    'String','t = 0.00 s', ...
    'BackgroundColor',[0.94 0.96 0.99],'FontSize',11,'FontWeight','bold', ...
    'HorizontalAlignment','left','ForegroundColor',[0.08 0.18 0.45]);

%%  Run / Reset buttons
btn_reset = uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.572 0.030 0.19 0.058],'String','Reset','FontSize',10, ...
    'FontWeight','bold','BackgroundColor',[0.90 0.55 0.45], ...
    'ForegroundColor','white');

btn_run = uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.775 0.030 0.19 0.058],'String','Run','FontSize',10, ...
    'FontWeight','bold','BackgroundColor',[0.25 0.65 0.35], ...
    'ForegroundColor','white');

%%  Initial aircraft draw 
hac   = make_aircraft(ax, eye(3), [0;0;0], geom);
htraj = plot3(ax,0,0,0,'b-','LineWidth',1.2);

%%  Button callbacks
btn_reset.Callback = @(s,e) do_reset(fig,sliders,slbls,jxz_chk, ...
                                      hac,htraj,ax,state_txt,time_txt, ...
                                      geom,x0,p0,HW);
btn_run.Callback = @(s,e) toggle_run(fig,btn_run,sliders,hac,htraj, ...
                                      ax,state_txt,time_txt,geom,dt,HW);
end


%% =========================================================================
function update_slider_labels(sliders,slbls)
    for k = 1:6
        slbls{k}.String = sprintf('%.0f', sliders{k}.Value);
    end
end

%% =========================================================================
function update_p_appdata(fig, jxz_on, p0, ~)
    p = p0;
    if jxz_on
        % Use a large Jxz to make roll-yaw coupling clearly visible.
        % Cessna 620 data gives Jxz=0; we inject a representative value
        % (~30% of Jx) so the effect is observable in the animation.
        p.Ixz = 0.10 * p0.Ixx;
    else
        p.Ixz = 0;
    end
    p = compute_gamma(p);
    setappdata(fig,'p',p);
end

%% =========================================================================
function toggle_run(fig,btn_run,sliders,hac,htraj,ax,state_txt,time_txt,geom,dt,HW)
    if ~ishandle(fig), return; end

    if getappdata(fig,'running')
        setappdata(fig,'running',0);
        btn_run.String           = 'Run';
        btn_run.BackgroundColor  = [0.25 0.65 0.35];
        btn_run.ForegroundColor  = 'white';
        return;
    end

    setappdata(fig,'running',1);
    btn_run.String          = 'Stop';
    btn_run.BackgroundColor = [0.80 0.20 0.20];
    btn_run.ForegroundColor = 'white';

    while ishandle(fig) && getappdata(fig,'running')
        step_clock = tic;

        p     = getappdata(fig,'p');
        u_in  = [sliders{1}.Value; sliders{2}.Value; sliders{3}.Value;
                 sliders{4}.Value; sliders{5}.Value; sliders{6}.Value];

        x_cur = getappdata(fig,'x_cur');
        t_cur = getappdata(fig,'t_cur');

        [~,xout] = ode45(@(t,x) eom(t,x,u_in,p), [t_cur, t_cur+dt], x_cur);
        x_cur    = xout(end,:)';
        t_cur    = t_cur + dt;

        % Wrap Euler angles
        x_cur(4) = wrapToPi(x_cur(4));
        x_cur(5) = wrapToPi(x_cur(5));
        x_cur(6) = wrapToPi(x_cur(6));

        setappdata(fig,'x_cur',x_cur);
        setappdata(fig,'t_cur',t_cur);

        % Update animation
        pos = x_cur(7:9);
        R   = make_dcm(x_cur(4), x_cur(5), x_cur(6));
        update_aircraft(hac, R, pos, geom);

        % Update trajectory trail
        tx = getappdata(fig,'traj_x');
        ty = getappdata(fig,'traj_y');
        tz = getappdata(fig,'traj_z');
        tx(end+1) = pos(1);
        ty(end+1) = pos(2);
        tz(end+1) = pos(3);
        setappdata(fig,'traj_x',tx);
        setappdata(fig,'traj_y',ty);
        setappdata(fig,'traj_z',tz);
        set(htraj,'XData',tx,'YData',ty,'ZData',tz);

        % Follow camera
        set(ax,'XLim',pos(1)+[-HW HW], ...
               'YLim',pos(2)+[-HW HW], ...
               'ZLim',pos(3)+[-HW HW]);

        update_state_display(state_txt, x_cur);
        time_txt.String = sprintf('t = %.2f s', t_cur);
        drawnow;

        elapsed = toc(step_clock);
        if dt - elapsed > 0, pause(dt - elapsed); end
    end

    if ishandle(fig)
        btn_run.String          = 'Run';
        btn_run.BackgroundColor = [0.25 0.65 0.35];
        btn_run.ForegroundColor = 'white';
        setappdata(fig,'running',0);
    end
end

%% =========================================================================
function do_reset(fig,sliders,slbls,jxz_chk,hac,htraj,ax,state_txt,time_txt,geom,x0,p0,HW)
    if ~ishandle(fig), return; end
    setappdata(fig,'running',0);
    drawnow; pause(0.05);

    setappdata(fig,'x_cur',x0);
    setappdata(fig,'t_cur',0);
    setappdata(fig,'traj_x',0);
    setappdata(fig,'traj_y',0);
    setappdata(fig,'traj_z',0);
    setappdata(fig,'p',p0);

    for k = 1:6
        sliders{k}.Value = 0;
        slbls{k}.String  = '0';
    end
    jxz_chk.Value = 0;

    set(htraj,'XData',0,'YData',0,'ZData',0);
    time_txt.String = 't = 0.00 s';
    update_aircraft(hac, eye(3), x0(7:9), geom);
    set(ax,'XLim',[-HW HW],'YLim',[-HW HW],'ZLim',[-HW HW]);
    update_state_display(state_txt, x0);
    drawnow;
end


%% =========================================================================
function xdot = eom(~, x, u, p)
% 6DOF EOM — no gravity, no initial velocity.
% Applied forces/moments only (equations 3.14–3.16, Beard & McLain).
    ub=x(1); vb=x(2); wb=x(3);
    phi=x(4); theta=x(5); psi=x(6);
    pr=x(10); qr=x(11); rr=x(12);

    Fx=u(1); Fy=u(2); Fz=u(3);
    l=u(4);  mm=u(5); n=u(6);

    % Translational dynamics (eq 3.14)
    udot = rr*vb - qr*wb + Fx/p.m;
    vdot = pr*wb - rr*ub + Fy/p.m;
    wdot = qr*ub - pr*vb + Fz/p.m;

    % Rotational dynamics (eq 3.16)
    pdot = p.G1*pr*qr - p.G2*qr*rr + p.G3*l  + p.G4*n;
    qdot = p.G5*pr*rr - p.G6*(pr^2-rr^2)      + mm/p.Iyy;
    rdot = p.G7*pr*qr - p.G1*qr*rr + p.G4*l  + p.G8*n;

    % Euler angle kinematics (eq 3.15)
    phidot   = pr + (qr*sin(phi) + rr*cos(phi))*tan(theta);
    thetadot = qr*cos(phi) - rr*sin(phi);
    psidot   = (qr*sin(phi) + rr*cos(phi)) / cos(theta);

    % Position kinematics (eq 3.13) — inertial frame
    cp=cos(phi); sp=sin(phi);
    ct=cos(theta); st=sin(theta);
    cs=cos(psi);  ss=sin(psi);

    xedot = (ct*cs)*ub + (sp*st*cs - cp*ss)*vb + (cp*st*cs + sp*ss)*wb;
    yedot = (ct*ss)*ub + (sp*st*ss + cp*cs)*vb + (cp*st*ss - sp*cs)*wb;
    zedot = (-st)*ub   + (sp*ct)*vb             + (cp*ct)*wb;

    xdot = [udot;vdot;wdot; phidot;thetadot;psidot; xedot;yedot;zedot; pdot;qdot;rdot];
end


%% =========================================================================
function p = compute_gamma(p)
    G    = p.Ixx*p.Izz - p.Ixz^2;
    p.G1 = p.Ixz*(p.Ixx - p.Iyy + p.Izz) / G;
    p.G2 = (p.Izz*(p.Izz - p.Iyy) + p.Ixz^2) / G;
    p.G3 = p.Izz / G;
    p.G4 = p.Ixz / G;
    p.G5 = (p.Izz - p.Ixx) / p.Iyy;
    p.G6 = p.Ixz / p.Iyy;
    p.G7 = ((p.Ixx - p.Iyy)*p.Ixx + p.Ixz^2) / G;
    p.G8 = p.Ixx / G;
end


%% =========================================================================
function update_state_display(state_txt, x)
    vals = [x(1:3); rad2deg(x(4:6)); x(7:9); rad2deg(x(10:12))];
    for k = 1:12
        state_txt{k}.String = sprintf('%.2f', vals(k));
    end
end


%% =========================================================================
function geom = make_geom(sc)
    geom.fus   = sc*[-4 0 0; 4 0 0];
    geom.wing  = sc*[0.5 6 0; -1.5 6 0; -1.5 -6 0; 0.5 -6 0];
    geom.htail = sc*[-3.5 2 0; -2.5 2 0; -2.5 -2 0; -3.5 -2 0];
    geom.vtail = sc*[-3.5 0 0; -2.5 0 0; -2.5 0 -1.8; -3.5 0 -1.4];
    th         = linspace(0,2*pi,20)';
    geom.body  = sc*[4*cos(th), zeros(20,1), 0.6*sin(th)];
end


%% =========================================================================
function h = make_aircraft(ax, R, t, geom)
    c_wing = [0.42 0.62 0.84];
    c_body = [0.55 0.72 0.90];
    c_tail = [0.60 0.65 0.80];
    c_edge = [0.18 0.28 0.52];

    fw   = xf(geom.wing,  R, t);
    fht  = xf(geom.htail, R, t);
    fvt  = xf(geom.vtail, R, t);
    fbod = xf(geom.body,  R, t);
    ffus = xf(geom.fus,   R, t);

    h.wing  = fill3(ax,fw(:,1),  fw(:,2),  fw(:,3),  c_wing,'FaceAlpha',0.75,'EdgeColor',c_edge,'LineWidth',1.0);
    h.htail = fill3(ax,fht(:,1), fht(:,2), fht(:,3), c_wing,'FaceAlpha',0.75,'EdgeColor',c_edge,'LineWidth',1.0);
    h.vtail = fill3(ax,fvt(:,1), fvt(:,2), fvt(:,3), c_tail,'FaceAlpha',0.75,'EdgeColor',c_edge,'LineWidth',1.0);
    h.body  = fill3(ax,fbod(:,1),fbod(:,2),fbod(:,3),c_body,'FaceAlpha',0.60,'EdgeColor',c_edge,'LineWidth',0.5);
    h.fus   = plot3(ax,ffus(:,1),ffus(:,2),ffus(:,3),'-','Color',c_edge,'LineWidth',2.5);

    cg = xf([0 0 0],   R, t);
    sc = norm(geom.fus(2,:) - geom.fus(1,:)) * 0.375;
    aX = xf([sc 0 0], R, t);
    aY = xf([0 sc 0], R, t);
    aZ = xf([0 0 sc], R, t);

    h.ax_x = plot3(ax,[cg(1) aX(1)],[cg(2) aX(2)],[cg(3) aX(3)],'r-','LineWidth',2);
    h.ax_y = plot3(ax,[cg(1) aY(1)],[cg(2) aY(2)],[cg(3) aY(3)],'-','Color',[0.1 0.6 0.1],'LineWidth',2);
    h.ax_z = plot3(ax,[cg(1) aZ(1)],[cg(2) aZ(2)],[cg(3) aZ(3)],'b-','LineWidth',2);
    h.lx   = text(ax,aX(1),aX(2),aX(3),' x_b','Color','r','FontSize',8,'FontWeight','bold');
    h.ly   = text(ax,aY(1),aY(2),aY(3),' y_b','Color',[0.1 0.6 0.1],'FontSize',8,'FontWeight','bold');
    h.lz   = text(ax,aZ(1),aZ(2),aZ(3),' z_b','Color','b','FontSize',8,'FontWeight','bold');
end


%% =========================================================================
function update_aircraft(h, R, t, geom)
    fw   = xf(geom.wing,  R, t);
    fht  = xf(geom.htail, R, t);
    fvt  = xf(geom.vtail, R, t);
    fbod = xf(geom.body,  R, t);
    ffus = xf(geom.fus,   R, t);

    set(h.wing, 'XData',fw(:,1),  'YData',fw(:,2),  'ZData',fw(:,3));
    set(h.htail,'XData',fht(:,1), 'YData',fht(:,2), 'ZData',fht(:,3));
    set(h.vtail,'XData',fvt(:,1), 'YData',fvt(:,2), 'ZData',fvt(:,3));
    set(h.body, 'XData',fbod(:,1),'YData',fbod(:,2),'ZData',fbod(:,3));
    set(h.fus,  'XData',ffus(:,1),'YData',ffus(:,2),'ZData',ffus(:,3));

    sc = norm(geom.fus(2,:) - geom.fus(1,:)) * 0.375;
    cg = xf([0 0 0],     R, t);
    aX = xf([sc 0 0],    R, t);
    aY = xf([0 sc 0],    R, t);
    aZ = xf([0 0 sc],    R, t);

    set(h.ax_x,'XData',[cg(1) aX(1)],'YData',[cg(2) aX(2)],'ZData',[cg(3) aX(3)]);
    set(h.ax_y,'XData',[cg(1) aY(1)],'YData',[cg(2) aY(2)],'ZData',[cg(3) aY(3)]);
    set(h.ax_z,'XData',[cg(1) aZ(1)],'YData',[cg(2) aZ(2)],'ZData',[cg(3) aZ(3)]);
    set(h.lx,'Position',aX);
    set(h.ly,'Position',aY);
    set(h.lz,'Position',aZ);
end


%% =========================================================================
function pts_w = xf(pts, R, t)
    pts_w = (R * pts' + t)';
end


%% =========================================================================
function R = make_dcm(phi, theta, psi)
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R  = Rz * Ry * Rx;
end