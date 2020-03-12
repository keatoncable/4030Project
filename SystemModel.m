clear
clc
close all

%% Constant Initialization
ts = 0.2; %impulse duration in seconds
tau = 1; %delay in seconds

%% Plant Model
t_num = [tau];
t_den = [1 tau];
timedelay = tf(t_num,t_den); %motor transfer function
p_num = [1 1];
p_den = [1 1 1];
plant = tf(p_num,p_den); %treadmill and cylinder function
d_num = [1 1];
d_den = [1 1 1];
disturb = tf(d_num,d_den); %disturbance transfer function

%% Input Magnitudes
open_vec = [24 0 1 2]; %step input value for open loop response
dist_vec = [0 10 2 4]; %N, disturbance force

min = -0.5; %m, minimum cylinder distance from relative center
max = 0.5; %m, maximum cylinder distance from relative center
typ = 0; %m, typical desired cylinder position at origin

%% Open Loop System Response
ystore = [];
for i = 1:length(open_vec)
    open_in = open_vec(i);
    dist_in = dist_vec(i);
    load_system('openloop')
    simOut = sim('openloop');
    y = simOut.get('ScopeData');
    ystore = [ystore y];
end

figure
hold on
plot(ystore(:,1),ystore(:,2))
plot(ystore(:,3),ystore(:,4))
plot(ystore(:,5),ystore(:,6))
plot(ystore(:,7),ystore(:,8))
ylabel('Output Value')
xlabel('Time (s)')
title('Open Loop System Response')
legend('Step = 1, Disturbance = 0','Step = 0, Disturbance = 1', 'Step = 1, Disturbance = 2','Step = 2, Disturbance = 4')

