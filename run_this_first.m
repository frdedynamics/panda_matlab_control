clear all; clc;


%% User interface
UI = [];
UI.simulation = 1;
UI.port = 11311;

%% User eval
if UI.simulation
    UI.ros_master_ip = 'localhost';
else
    UI.ros_master_ip = '172.31.1.21';
end

UI.masterURI = "http://" + UI.ros_master_ip + ":" + UI.port;

rosinit(UI.masterURI)



