clear
clc
close all

%% Plant Model
m_num = [1 1];
m_den = [1 1 1];
motor = tf(m_num,m_den); %motor transfer function
p_num = [1 1];
p_den = [1 1 1];
plant = tf(p_num,p_den); %treadmill and cylinder function
k = 1; %controller gain
g_num = [1 1];
g_den = [1 1 1];
gain = k*tf(g_num,g_den); %controller gain for lead\lag
d_num = [1 1];
d_den = [1 1 1];
disturb = tf(d_num,d_den); %disturbance transfer function

%% Input Magnitudes
open_vec = [1 0 1 2]; %step input value for open loop response
dist_vec = [0 1 2 4]; %N, disturbance force

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

