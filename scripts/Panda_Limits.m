%% Limits
HW.Joint.VelocityLimit = NaN(7,2); % [rad/s]
for i=1:4
    HW.Joint.VelocityLimit(i,2) = 2.1750;
    HW.Joint.VelocityLimit(i,1) = -2.1750;
end
for i=5:7
    HW.Joint.VelocityLimit(i,2) = 2.6100;
    HW.Joint.VelocityLimit(i,1) = -2.6100;
end

HW.Joint.AccelLimit = NaN(7,2); % [rad/s^2]
HW.Joint.AccelLimit([1,5],2) = 15;
HW.Joint.AccelLimit([1,5],1) = -15;

HW.Joint.AccelLimit(2,2) = 7.5;
HW.Joint.AccelLimit(2,1) = -7.5;

HW.Joint.AccelLimit(3,2) = 10;
HW.Joint.AccelLimit(3,1) = -10;

HW.Joint.AccelLimit(4,2) = 12.5;
HW.Joint.AccelLimit(4,1) = -12.5;

HW.Joint.AccelLimit([6,7],2) = 20;
HW.Joint.AccelLimit([6,7],1) = -20;

HW.Joint.JerkLimit = NaN(7,2); % [rad/s^3]
HW.Joint.JerkLimit([1,5],2) = 7500;
HW.Joint.JerkLimit([1,5],1) = -7500;

HW.Joint.JerkLimit(2,2) = 3750;
HW.Joint.JerkLimit(2,1) = -3750;

HW.Joint.JerkLimit(3,2) = 5000;
HW.Joint.JerkLimit(3,1) = -5000;

HW.Joint.JerkLimit(4,2) = 6250;
HW.Joint.JerkLimit(4,1) = -6250;

HW.Joint.JerkLimit([6,7],2) = 10000;
HW.Joint.JerkLimit([6,7],1) = -10000;

HW.Joint.TorqueLimit = NaN(7,2); % [Nm]
for i=1:4
    HW.Joint.TorqueLimit(i,2) = 87;
    HW.Joint.TorqueLimit(i,1) = -87;
end
for i=5:7
    HW.Joint.TorqueLimit(i,2) = 12;
    HW.Joint.TorqueLimit(i,1) = -12;
end

HW.Joint.TorqueDerivativeLimit = ones(7,2).*[-1000,1000]; % [Nm/s]