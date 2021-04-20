ROS_STA = [];
ROS_STA.ros_already_running = 0;
ROS_STA.ros_started = 0;
ROS_STA.ros_running = 0;
ROS_STA.no_local_ros_core = 0;
ROS_STA.current_port = NaN;
ROS_STA.current_ros_master_ip = NaN;

try 
    rosnode list
    ROS_STA.ros_already_running = 1;
catch
    if UI.simulation && (~exist('core','var') || isempty(core) || ~isvalid(core))
        core = ros.Core(UI.port);
    end
    try
        rosinit(UI.masterURI)
    catch
        warning('Oopsie, rosshutdown then rosinit again..')
        rosshutdown
        rosinit(UI.masterURI)
    end
    ROS_STA.ros_started = 1;
end


if ROS_STA.ros_already_running || ROS_STA.ros_started
    ROS_STA.ros_running = 1;
end

if ~exist('core','var') || isempty(core)
    ROS_STA.no_local_ros_core = 1;
    if UI.simulation
        error('Simulation selected, but no local core found.')
    end
    if ~ros.internal.NetworkIntrospection.isMasterReachable(UI.masterURI)
        error('Will the real ros master please stand up?')
    else
        [ROS_STA.current_ros_master_ip, ROS_STA.current_port] = ...
            ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(UI.masterURI);
    end
else
    ROS_STA.no_local_ros_core = 0;
    [ROS_STA.current_ros_master_ip, ROS_STA.current_port] = ...
            ros.internal.NetworkIntrospection.getHostAndPortFromMasterURI(core.MasterURI);
    if UI.simulation
        if strcmp(ROS_STA.current_ros_master_ip, UI.real_ros_master_ip)
            error('Simulation selected, but ROS master ip is same as core.')
        end
        clear tmp
    end
end

ROS_STA
