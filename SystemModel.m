clear
clc
close all

%% Constant Initialization
dt = 0.2; %impulse duration in seconds
tau = 0.5; %delay in seconds

r = 0.0635; %radius of rollers
kt = 0.15556; %motor constant
L = 2.9*10^-3; %motor inductance in H
R = 1.4; %motor resistance in R
ke = 0.155556;
m_p = 10; %pulley mass in kg
m_b = 2; %belt mass in kg
b = 0.01*2; %motor bearing friction
Je = m_p*r^2 + m_b*r^2+40*10^-6; %effective inertia

%% Plant Model
t_num = [tau];
t_den = [1 tau];
timedelay = tf(t_num,t_den); %motor transfer function
p_num = [kt];
p_den = [L*Je L*b+R*Je+kt*ke R*b];
plant = tf(p_num,p_den); %treadmill and cylinder function
d_num = [L*r R*r];
d_den = [L*Je L*b+R*Je+kt*ke R*b];
disturb = tf(d_num,d_den); %disturbance transfer function

%% Input Magnitudes
volt_vec = 24;%[0 12 24 36 48]; %step input value for open loop response

theta = 0; %treadmill incline
mass = [50 60 70 80 90 100 110 120 2000]; %person weight in kg
mu = 0.4; %coefficient of friction
g = 9.81;
fd_vec = -mu*g*cosd(theta).*mass; %N, disturbance force

%% Open Loop System Response
ystore = {};
for i = 1:length(volt_vec)
    volt = volt_vec(i);
    fd = fd_vec(4);
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

