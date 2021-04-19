fignum = 1;

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
tiled_ = tiledlayout(2,7, 'Padding', 'compact', 'TileSpacing', 'compact');
tiled_.Title.String = 'Planned joint path';
tiled_.Title.FontSize = 15;

for i=1:7
    nexttile(i)
    plot(UI.t, qd(:,i), 'LineWidth', 1.5)
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
    plot(UI.t(2:end), diff(qd(:,i))/UI.timestep, 'LineWidth', 1.5)
    hold on
    grid on
   	plot(UI.t(2:end), ...
        ones(length(UI.t(2:end)),1)*UI.max_joint_change(i), ...
        'r--', 'LineWidth', 1.5)
    plot(UI.t(2:end), ...
        ones(length(UI.t(2:end)),1)*UI.max_joint_change(i), ...
        'r--', 'LineWidth', 1.5)
    ylabel("J" + i + " [rad/s]")
    
    if i==4
        xlabel('t [s]')
    end
end