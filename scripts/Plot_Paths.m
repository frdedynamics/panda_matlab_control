h = figure(1); clf;
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