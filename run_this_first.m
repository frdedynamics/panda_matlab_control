clear all; clc;

%% Get HW constraints
run ./scripts/Panda_Limits.m

%% User interface
UI = [];
UI.simulation = 1;
UI.port = 11311;

UI.timestep = 0.001;
UI.motion_duration = 20;

UI.max_joint_change = 90*pi/180*ones(7,1); % rad/s

% User eval
if UI.simulation
    UI.ros_master_ip = 'localhost';
else
    UI.ros_master_ip = '172.31.1.21';
end

for i=1:7
   if UI.max_joint_change(i) > HW.Joint.VelocityLimit(i,2)
       warning('max_joint_change no good. Setting to max. Check HW limits.')
       UI.max_joint_change(i) = HW.Joint.VelocityLimit(i,2);
   end
   if UI.max_joint_change(i) < 0
       error('max_joint_change negative. wtf. Check HW limits.')
       
   end
end

UI.masterURI = "http://" + UI.ros_master_ip + ":" + UI.port;

UI.t = 0:UI.timestep:(UI.motion_duration - UI.timestep);

%% Add paths
addpath ./functions

%% Get RBT
[robot, robotData] = loadrobot('frankaEmikaPanda',...
    'DataFormat', 'row',...
    'Gravity', [0, 0, -9.80665]);
removeBody(robot,'panda_leftfinger');
removeBody(robot,'panda_rightfinger');
tmp_panda_fingertipcenter = rigidBody('panda_fingertipcenter');
setFixedTransform(tmp_panda_fingertipcenter.Joint, [0 pi -0.1 0], 'dh');
addBody(robot,tmp_panda_fingertipcenter,'panda_hand');

%     interactiveRigidBodyTree(robot);

clear tmp_panda_fingertipcenter

%% Get data
load mean_var_linconstraint_rotated_z.mat

m_interpolated = interp1(1:length(m), m, linspace(1,length(m),UI.motion_duration/UI.timestep));
v_interpolated = interp1(1:length(v), v, linspace(1,length(v),UI.motion_duration/UI.timestep));

XYZ_path = m_interpolated(:,1:3);
Quat_path = m_interpolated(:,4:7);
Euler_path = quat2eul(Quat_path);

%% Sanity check
warning('No sanity.')

%% Transformation matrix
Transformations = zeros(4,4,length(m_interpolated));
for i=1:length(m_interpolated)
    Transformations(1:3,4,i) = XYZ_path(i,:)';
    Transformations(1:3,1:3,i) = quat2rotm(Quat_path(i,:));
    Transformations(4,4,i) = 1;
end

%% ik and get q0
ik = inverseKinematics('RigidBodyTree', robot);
[q0,solnInfo] = ik('panda_fingertipcenter', Transformations(:,:,1), ...
    [0.3 0.3 0.3 0.9 0.9 0.9], robot.homeConfiguration);

if solnInfo.ExitFlag~=1 || ~strcmp(solnInfo.Status, 'success')
    warning('q0 not found correctly.')
end
q0 = [0 -0.1 0 -1.9 0 2.0 0];
%% gik
gik = generalizedInverseKinematics('RigidBodyTree',robot);
gik.ConstraintInputs = {'joint','pose'};
limitJointChange = constraintJointBounds(robot);
limitJointChange.Weights = [1 1 1 1 1 1 1];
hw_joint_bounds = limitJointChange.Bounds;

poseTgt = constraintPoseTarget('panda_fingertipcenter');
poseTgt.OrientationTolerance = 1*pi/180;
poseTgt.PositionTolerance = 0.1/100; 
poseTgt.Weights = [0.9 0.9]; %[Orientation Position]

%% Joint space trajectory

qd = NaN(length(m_interpolated),7);
disp('Inverse kinematics..')
tmp = length(m_interpolated);
tmp2 = zeros(length(m_interpolated),1);
tic
ik_step_size = 20;
for i=1:ik_step_size:length(m_interpolated)
    if toc > 30
        100*i/tmp
        tic
    end
    
    if i==1
        initialguess = q0;
    elseif solnInfo.ExitFlag==1 && strcmp(solnInfo.Status, 'success') || 1
        initialguess = qd(i-ik_step_size,:);
    else
        % NO change
    end
    
    poseTgt.TargetTransform = Transformations(:,:,i);
    if i>1
        limitJointChange.Bounds = update_joint_limits(...
            hw_joint_bounds, ...
            UI.max_joint_change*UI.timestep*ik_step_size, ...
            qd(i-ik_step_size,:));
    end
    
    [configSoln, solnInfo] = gik(initialguess,limitJointChange,poseTgt);
    qd(i,:) = configSoln;
    if solnInfo.ExitFlag~=1
        tmp2(i) = 1;
    end
    
end
disp('Inverse kinematics done.')

%%
qd = qd(1:ik_step_size:length(m_interpolated),:);

kmi = zeros(length(qd),1);
for i=1:length(qd)
    J = robot.geometricJacobian(qd(i,:), 'panda_fingertipcenter');
    kmi(i) = max(0, det(J * J'));
end

if ik_step_size > 1
    qd_interpolated = interp1(1:length(qd), qd, linspace(1,length(qd),length(m_interpolated)));
else
    qd_interpolated = qd;
end

clear tmp_qd

%% Null space 

%% Plot
run ./scripts/Plot_Paths.m

%% ROS
%rosinit(UI.masterURI)


