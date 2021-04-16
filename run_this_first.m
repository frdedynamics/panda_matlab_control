clear all; clc;


%% User interface
UI = [];
UI.simulation = 1;
UI.port = 11311;

UI.timestep = 0.001;
UI.motion_duration = 12;

% User eval
if UI.simulation
    UI.ros_master_ip = 'localhost';
else
    UI.ros_master_ip = '172.31.1.21';
end

UI.masterURI = "http://" + UI.ros_master_ip + ":" + UI.port;

UI.t = 0:UI.timestep:UI.motion_duration;

%% Get RBT
[robot, robotData] = loadrobot('frankaEmikaPanda',...
    'DataFormat', 'column',...
    'Gravity', [0, 0, -9.80665]);
removeBody(robot,'panda_leftfinger');
removeBody(robot,'panda_rightfinger');
panda_fingertipcenter = rigidBody('panda_fingertipcenter');
setFixedTransform(panda_fingertipcenter.Joint, [0 pi -0.1 0], 'dh');
addBody(robot,panda_fingertipcenter,'panda_hand');
%     interactiveRigidBodyTree(robot);

%% Get data
load mean_var_linconstraint_rotated_z.mat

%
m_interpolated = interp1(1:length(m), m, linspace(1,length(m),UI.motion_duration/UI.timestep));
v_interpolated = interp1(1:length(v), v, linspace(1,length(v),UI.motion_duration/UI.timestep));

% Sanity check
warning('No sanity.')

%% Task space trajectory

%% Joint space trajectory

%% Null space 

%% ROS
rosinit(UI.masterURI)



