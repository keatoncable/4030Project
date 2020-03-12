clear
clc
close all

%% Constant Initialization
dt = 0.2; %impulse duration in seconds
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
volt_vec = [24 0 12 24 12 24]; %step input value for open loop response
fd_vec = [0 10 10 20 -10 -20]; %N, disturbance force

%% Open Loop System Response
ystore = {};
for i = 1:length(volt_vec)
    volt = volt_vec(i);
    fd = fd_vec(i);
    load_system('openloop')
    simOut = sim('openloop');
    y = simOut.get('ScopeData');
    ystore = [ystore y];
end

fig1 = figure;
hold on
plot(ystore{1}(:,1),ystore{1}(:,2))
plot(ystore{2}(:,1),ystore{2}(:,2))
plot(ystore{3}(:,1),ystore{3}(:,2))
plot(ystore{4}(:,1),ystore{4}(:,2))
plot(ystore{5}(:,1),ystore{5}(:,2))
plot(ystore{6}(:,1),ystore{6}(:,2))
ylabel('Cylinder Position')
xlabel('Time (s)')
title('Open Loop System Response')
legend('V = 24, Fd = 0','V = 0, Fd = 10', 'V = 12, Fd = 10','V = 24, Fd = 20','V = 12, Fd = -10','V = 24, Fd = -20')
saveas(fig1,'Open Loop System Respone.jpg')

for i = 1:length(ystore)
    fig = figure;
    plot(ystore{i}(:,1),ystore{i}(:,2))
    ylabel('Cylinder Position')
    xlabel('Time (s)')
    tstr = sprintf('Voltage = %.0f V, Fd = %.0f N',volt_vec(i),fd_vec(i));
    sstr = sprintf('V%.0f__Fd%.0f.jpg',volt_vec(i),fd_vec(i));
    title(tstr)
    saveas(fig,sstr)
end

