function rotation_translation_demo()
% rotation_translation_demo
%
% AE 700 - Section 1: Coordinate Frames
%
% LEFT  : p_world = R * p_body + t   (rotate THEN translate)
% RIGHT : p_world = R * (p_body + t) (translate THEN rotate)
%
% Run by typing:  rotation_translation_demo

close all;

% aircraft geometry in body frame
fus_pts = [-4 0 0; 4 0 0];

wing_pts = [ 0.5   6   0;
            -1.5   6   0;
            -1.5  -6   0;
             0.5  -6   0];

htail_pts = [-3.5   2   0;
             -2.5   2   0;
             -2.5  -2   0;
             -3.5  -2   0];

vtail_pts = [-3.5   0    0;
             -2.5   0    0;
             -2.5   0   -1.8;
             -3.5   0   -1.4];

th       = linspace(0, 2*pi, 20)';
body_pts = [4*cos(th), zeros(20,1), 0.6*sin(th)];

geom.fus   = fus_pts;
geom.wing  = wing_pts;
geom.htail = htail_pts;
geom.vtail = vtail_pts;
geom.body  = body_pts;

% Figure: white background,
fig = figure('Name','Rotation vs Translation Order', ...
             'Position',[80 80 1200 620], ...
             'Color','white');

% Left axes
ax1 = axes('Parent',fig, 'Position',[0.04 0.32 0.44 0.63]);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
ax1.Color           = 'white';
ax1.GridColor       = [0.80 0.80 0.80];
ax1.GridAlpha       = 0.6;
ax1.XColor          = [0.25 0.25 0.25];
ax1.YColor          = [0.25 0.25 0.25];
ax1.ZColor          = [0.25 0.25 0.25];
ax1.FontSize        = 9;
ax1.LineWidth       = 0.8;
ax1.Box             = 'on';

title(ax1, {'Order A:  p_{world} = R \cdot p_{body} + t', ...
            'Rotate about CG, then translate'}, ...
      'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.10 0.35 0.10]);

xlabel(ax1,'X  (North)','FontSize',9);
ylabel(ax1,'Y  (East)', 'FontSize',9);
zlabel(ax1,'Z  (Down)', 'FontSize',9);
view(ax1, 35, 25);
xlim(ax1,[-12 12]); ylim(ax1,[-8 8]); zlim(ax1,[-8 8]);
set(ax1,'ZDir','reverse','YDir','reverse');

% World origin marker
plot3(ax1,0,0,0,'r.','MarkerSize',16);
text(ax1,0.4,0.4,0,'World origin','FontSize',8,'Color',[0.7 0.1 0.1]);

% Right axes 
ax2 = axes('Parent',fig, 'Position',[0.54 0.32 0.44 0.63]);
hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
ax2.Color           = 'white';
ax2.GridColor       = [0.80 0.80 0.80];
ax2.GridAlpha       = 0.6;
ax2.XColor          = [0.25 0.25 0.25];
ax2.YColor          = [0.25 0.25 0.25];
ax2.ZColor          = [0.25 0.25 0.25];
ax2.FontSize        = 9;
ax2.LineWidth       = 0.8;
ax2.Box             = 'on';

title(ax2, {'Order B:  p_{world} = R \cdot (p_{body} + t)', ...
            'Translate CG, then rotate about CG'}, ...
      'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.30 0.10 0.10]);

xlabel(ax2,'X  (North)','FontSize',9);
ylabel(ax2,'Y  (East)', 'FontSize',9);
zlabel(ax2,'Z  (Down)', 'FontSize',9);
view(ax2, 35, 25);
xlim(ax2,[-12 12]); ylim(ax2,[-8 8]); zlim(ax2,[-8 8]);
set(ax2,'ZDir','reverse','YDir','reverse');

plot3(ax2,0,0,0,'r.','MarkerSize',16);
text(ax2,0.4,0.4,0,'World origin','FontSize',8,'Color',[0.7 0.1 0.1]);

% Initial draw
R0 = eye(3);
h1 = make_aircraft(ax1, R0, [5;0;0], false, geom);
h2 = make_aircraft(ax2, R0, [5;0;0], true,  geom);

%% Sliders
% Row positions (normalized, bottom-up)
ypos  = [0.255 0.205 0.155 0.105 0.055 0.005];
names = {'Roll  \phi (deg)', 'Pitch  \theta (deg)', 'Yaw  \psi (deg)', ...
         'X offset', 'Y offset', 'Z offset'};
mins  = [-180  -80  -180  -8  -8  -8];
maxs  = [ 180   80   180   8   8   8];
vals  = [   0    0     0   5   0   0];
steps = {[1/360 10/360],[1/160 10/160],[1/360 10/360], ...
         [1/80 1/8],[1/80 1/8],[1/80 1/8]};

% Accent colours for each slider row
rowcols = [0.85 0.92 0.85;   % roll  – light green
           0.92 0.88 0.85;   % pitch – light orange
           0.85 0.88 0.95;   % yaw   – light blue
           0.95 0.95 0.95;   % X     – light grey
           0.95 0.95 0.95;   % Y
           0.95 0.95 0.95];  % Z

sliders = gobjects(6,1);
labels  = gobjects(6,1);

for k = 1:6
    % Row background panel
    uipanel(fig, 'Units','normalized', ...
        'Position',[0.01 ypos(k)-0.002 0.98 0.048], ...
        'BackgroundColor', rowcols(k,:), ...
        'BorderType','none');

    % Name label
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.02 ypos(k) 0.13 0.042], ...
        'String', names{k}, ...
        'BackgroundColor', rowcols(k,:), ...
        'HorizontalAlignment','left', ...
        'FontSize', 9, 'FontWeight','bold', ...
        'ForegroundColor',[0.20 0.20 0.20]);

    % Slider
    sliders(k) = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[0.16 ypos(k)+0.005 0.65 0.034], ...
        'Min',mins(k),'Max',maxs(k),'Value',vals(k), ...
        'SliderStep', steps{k}, ...
        'BackgroundColor',[0.88 0.88 0.88]);

    % Value label
    if k <= 3
        str = '  0 deg';
    else
        str = '0.0';
    end
    labels(k) = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.82 ypos(k) 0.16 0.042], ...
        'String', str, ...
        'BackgroundColor', rowcols(k,:), ...
        'FontSize', 10, 'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'ForegroundColor',[0.10 0.10 0.50]);
end

cb = @(s,e) update_cb(sliders, labels, h1, h2, geom);
for k = 1:6
    sliders(k).Callback = cb;
end

end % rotation_translation_demo


%% =========================================================================
function update_cb(sliders, labels, h1, h2, geom)
    phi_deg   = sliders(1).Value;
    theta_deg = sliders(2).Value;
    psi_deg   = sliders(3).Value;
    tx        = sliders(4).Value;
    ty        = sliders(5).Value;
    tz        = sliders(6).Value;

    labels(1).String = sprintf('%+.0f deg', phi_deg);
    labels(2).String = sprintf('%+.0f deg', theta_deg);
    labels(3).String = sprintf('%+.0f deg', psi_deg);
    labels(4).String = sprintf('%.1f',      tx);
    labels(5).String = sprintf('%.1f',      ty);
    labels(6).String = sprintf('%.1f',      tz);

    R = make_dcm(deg2rad(phi_deg), deg2rad(theta_deg), deg2rad(psi_deg));

    update_aircraft(h1, R, [tx;ty;tz], false, geom);
    update_aircraft(h2, R, [tx;ty;tz], true,  geom);
    drawnow limitrate;
end


%% =========================================================================
function h = make_aircraft(ax, R, t, translate_first, geom)
    % Colours: muted blue palette, works on white background
    c_wing  = [0.42 0.62 0.84];   % wing/htail fill
    c_body  = [0.55 0.72 0.90];   % fuselage fill
    c_tail  = [0.60 0.65 0.80];   % vtail fill
    c_edge  = [0.18 0.28 0.52];   % edges
    c_fus   = [0.18 0.28 0.52];   % fuselage line

    fw   = xf(geom.wing,  R, t, translate_first);
    fht  = xf(geom.htail, R, t, translate_first);
    fvt  = xf(geom.vtail, R, t, translate_first);
    fbod = xf(geom.body,  R, t, translate_first);
    ffus = xf(geom.fus,   R, t, translate_first);

    h.wing  = fill3(ax, fw(:,1),   fw(:,2),   fw(:,3),   c_wing, ...
        'FaceAlpha',0.75,'EdgeColor',c_edge,'LineWidth',1.0);
    h.htail = fill3(ax, fht(:,1),  fht(:,2),  fht(:,3),  c_wing, ...
        'FaceAlpha',0.75,'EdgeColor',c_edge,'LineWidth',1.0);
    h.vtail = fill3(ax, fvt(:,1),  fvt(:,2),  fvt(:,3),  c_tail, ...
        'FaceAlpha',0.75,'EdgeColor',c_edge,'LineWidth',1.0);
    h.body  = fill3(ax, fbod(:,1), fbod(:,2), fbod(:,3), c_body, ...
        'FaceAlpha',0.60,'EdgeColor',c_edge,'LineWidth',0.5);
    h.fus   = plot3(ax, ffus(:,1), ffus(:,2), ffus(:,3), '-', ...
        'Color',c_fus,'LineWidth',2.5);

    % Body-frame axes
    cg = xf([0 0 0],   R, t, translate_first);
    aX = xf([2.5 0 0], R, t, translate_first);
    aY = xf([0 2.5 0], R, t, translate_first);
    aZ = xf([0 0 2.5], R, t, translate_first);

    h.ax_x = plot3(ax,[cg(1) aX(1)],[cg(2) aX(2)],[cg(3) aX(3)], ...
        'r-','LineWidth',2.0);
    h.ax_y = plot3(ax,[cg(1) aY(1)],[cg(2) aY(2)],[cg(3) aY(3)], ...
        '-','Color',[0.1 0.6 0.1],'LineWidth',2.0);
    h.ax_z = plot3(ax,[cg(1) aZ(1)],[cg(2) aZ(2)],[cg(3) aZ(3)], ...
        'b-','LineWidth',2.0);
    h.lx   = text(ax,aX(1),aX(2),aX(3),' x_b', ...
        'Color','r','FontSize',8,'FontWeight','bold');
    h.ly   = text(ax,aY(1),aY(2),aY(3),' y_b', ...
        'Color',[0.1 0.6 0.1],'FontSize',8,'FontWeight','bold');
    h.lz   = text(ax,aZ(1),aZ(2),aZ(3),' z_b', ...
        'Color','b','FontSize',8,'FontWeight','bold');
end


%% =========================================================================
function update_aircraft(h, R, t, translate_first, geom)
    fw   = xf(geom.wing,  R, t, translate_first);
    fht  = xf(geom.htail, R, t, translate_first);
    fvt  = xf(geom.vtail, R, t, translate_first);
    fbod = xf(geom.body,  R, t, translate_first);
    ffus = xf(geom.fus,   R, t, translate_first);

    set(h.wing,  'XData',fw(:,1),   'YData',fw(:,2),   'ZData',fw(:,3));
    set(h.htail, 'XData',fht(:,1),  'YData',fht(:,2),  'ZData',fht(:,3));
    set(h.vtail, 'XData',fvt(:,1),  'YData',fvt(:,2),  'ZData',fvt(:,3));
    set(h.body,  'XData',fbod(:,1), 'YData',fbod(:,2), 'ZData',fbod(:,3));
    set(h.fus,   'XData',ffus(:,1), 'YData',ffus(:,2), 'ZData',ffus(:,3));

    cg = xf([0 0 0],   R, t, translate_first);
    aX = xf([2.5 0 0], R, t, translate_first);
    aY = xf([0 2.5 0], R, t, translate_first);
    aZ = xf([0 0 2.5], R, t, translate_first);

    set(h.ax_x,'XData',[cg(1) aX(1)],'YData',[cg(2) aX(2)],'ZData',[cg(3) aX(3)]);
    set(h.ax_y,'XData',[cg(1) aY(1)],'YData',[cg(2) aY(2)],'ZData',[cg(3) aY(3)]);
    set(h.ax_z,'XData',[cg(1) aZ(1)],'YData',[cg(2) aZ(2)],'ZData',[cg(3) aZ(3)]);
    set(h.lx,'Position',aX);
    set(h.ly,'Position',aY);
    set(h.lz,'Position',aZ);
end


%% =========================================================================
function pts_w = xf(pts, R, t, translate_first)
    p = pts';
    if translate_first
        % Order B: translate CG first, then rotate about new CG
        pts_w = (R * p + t)';
    else
        % Order A: rotate about CG first, then translate
        pts_w = (R * (p + t))';
    end
end


%% =========================================================================
function R = make_dcm(phi, theta, psi)
    Rx = [1      0         0     ;
          0  cos(phi)  -sin(phi) ;
          0  sin(phi)   cos(phi) ];
    Ry = [ cos(theta)  0  sin(theta);
           0           1  0         ;
          -sin(theta)  0  cos(theta)];
    Rz = [cos(psi) -sin(psi)  0;
          sin(psi)  cos(psi)  0;
          0         0         1];
    R  = Rz * Ry * Rx;
end