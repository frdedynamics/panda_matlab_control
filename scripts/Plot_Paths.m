fignum = 1;

xy_axis_font_size = 16;
legend_font_size = 20;

xy_axis_font_weight = 'bold';
legend_font_weight = xy_axis_font_weight;

%% Plot desired task space path
h = figure(fignum); clf;
ax = axes(h);
tiled_ = tiledlayout(2,2, 'Padding', 'compact', 'TileSpacing', 'compact');
tiled_.Title.String = 'Planned cartesian path';
tiled_.Title.FontSize = 15;


nexttile(1)
plot(UI.t, XYZ_path(:,1),'r')
hold on
plot(UI.t, XYZ_path(:,2),'b')
plot(UI.t, XYZ_path(:,3),'g')
ylabel('XYZ [m]')
xlabel('t [s]')
legend('x', 'y', 'z')

nexttile(3)
plot(UI.t(2:end), diff(XYZ_path(:,3))./UI.timestep,'r')
hold on
plot(UI.t(2:end), diff(XYZ_path(:,2))./UI.timestep,'b')
plot(UI.t(2:end), diff(XYZ_path(:,1))./UI.timestep,'g')
ylabel('XYZ [m/s]')
xlabel('t [s]')
legend('x', 'y', 'z')

nexttile(2)
plot(UI.t, Euler_path(:,1),'r')
hold on
plot(UI.t, Euler_path(:,2),'b')
plot(UI.t, Euler_path(:,3),'g')
ylabel('Euler [rad]')
xlabel('t [s]')
legend('x', 'y', 'z')

nexttile(4)
plot(UI.t(2:end), diff(Euler_path(:,3))./UI.timestep,'r')
hold on
plot(UI.t(2:end), diff(Euler_path(:,2))./UI.timestep,'b')
plot(UI.t(2:end), diff(Euler_path(:,1))./UI.timestep,'g')
ylabel('Euler [rad/s]')
xlabel('t [s]')
legend('x', 'y', 'z')

%%
fignum = fignum + 1;

%% Plot desired joint paths
h = figure(fignum); clf;
ax = axes(h);
tiled_ = tiledlayout(3,7, 'Padding', 'compact', 'TileSpacing', 'compact');
tiled_.Title.String = 'Planned joint path';
tiled_.Title.FontSize = 15;

hSub=nexttile(19);
plot(1, nan, ... 
    'r--', 'LineWidth', 1.5);
hold on;
plot(1, nan, ... 
    'k-.', 'LineWidth', 1.5);
set(hSub, 'Visible', 'off');
legend('Hardware limits','User limits', ...
    'interpreter', 'latex','FontSize', legend_font_size,...
        'FontWeight', legend_font_weight)%,...
    %'Location','southoutside')

for i=1:7
    nexttile(i)
    plot(UI.t, qd_interpolated(:,i), 'LineWidth', 1.5)
    hold on
    grid on
    plot(UI.t, ...
        ones(length(UI.t),1)*robot.Bodies{i}.Joint.PositionLimits(1), ...
        'r--', 'LineWidth', 1.5)
    plot(UI.t, ...
        ones(length(UI.t),1)*robot.Bodies{i}.Joint.PositionLimits(2), ...
        'r--', 'LineWidth', 1.5)
%     ylim(1.1*robot.Bodies{i}.Joint.PositionLimits)
    ylim([-7*pi/5,7*pi/5])
    ylabel("J" + i + " [rad]")
    

    nexttile(7+i)
    plot(UI.t(2:end), diff(qd_interpolated(:,i))/UI.timestep, 'LineWidth', 1.5)
    hold on
    grid on
   	plot(UI.t(2:end), ...
        ones(length(UI.t(2:end)),1)*UI.max_joint_change(i), ...
        'k-.', 'LineWidth', 1.5)
    plot(UI.t(2:end), ...
        ones(length(UI.t(2:end)),1)*-UI.max_joint_change(i), ...
        'k-.', 'LineWidth', 1.5)
    plot(UI.t(2:end), ...
        ones(length(UI.t(2:end)),1)*HW.Joint.VelocityLimit(i,1), ...
        'r--', 'LineWidth', 1.5)
    plot(UI.t(2:end), ...
        ones(length(UI.t(2:end)),1)*HW.Joint.VelocityLimit(i,2), ...
        'r--', 'LineWidth', 1.5)
    ylabel("J" + i + " [rad/s]")
    
    if i==4
        xlabel('t [s]')
    end
end

%% 
fignum = fignum + 1;

%%


h = figure(fignum); clf;
ax = axes(h);
tiled_ = tiledlayout(1,1, 'Padding', 'compact', 'TileSpacing', 'compact');
tiled_.Title.String = 'Manipulability';
tiled_.Title.FontSize = 15;

nexttile(1)
plot(UI.t(1:ik_step_size:length(UI.t)), kmi);
grid on


%% 
fignum = fignum + 1;

%% Visualize
h = figure(fignum); clf;

for i=1:10:length(qd)
    figure(fignum);
    show(robot,qd(i,:));
    
    pause(0.1)
end