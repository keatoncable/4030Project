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
open_in = 1; %step input value for open loop response
dist_in = 2; %N, disturbance force

min = -0.5; %m, minimum cylinder distance from relative center
max = 0.5; %m, maximum cylinder distance from relative center
typ = 0; %m, typical desired cylinder position at origin

%% Open Loop System Response
load_system('openloop')
simOut = sim('openloop');
y = simOut.get('ScopeData');
figure
plot(y(:,1),y(:,2))
ylabel('Output Value')
xlabel('Time (s)')
title('Open Loop System Response')

