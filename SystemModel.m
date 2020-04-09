clear
clc
close all

%% Constant Initialization
dt = 0.2; %impulse duration in seconds
tau = 2; %delay in seconds
dist_time = [10 25];

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
volt_vec = [0 12 24 48]; %step input value for open loop response

theta1 = 0; %min treadmill incline
theta2 = 45; %max treadmill incline
mass = [0 -50 -70 -90 -110]; %person weight in kg
mu = 0.4; %coefficient of friction
g = 9.81;
fd_vec = mu*g*cosd(theta1)*r.*mass; %N, disturbance force
fd_vec2 = mu*g*cosd(theta2)*r.*mass; %N, disturbance force

mtable = {'Person Mass [kg]' 'Disturbance [N], $\theta=45^\circ$'  'Disturbance [N], $\theta=45^\circ$';
            -mass(2) -fd_vec(2) -fd_vec2(2);
            -mass(3) -fd_vec(3) -fd_vec2(3);
            -mass(4) -fd_vec(4) -fd_vec2(4);
            -mass(5) -fd_vec(5) -fd_vec2(5);}

%% Open Loop No Disturbance
nodist = {};
load_system('openloop')
for i = 1:length(volt_vec)
    volt = volt_vec(i);
    fd = fd_vec(1);
    time = dist_time(1);
    simOut = sim('openloop');
    y = simOut.get('ScopeData');
    nodist = [nodist y];
end

fig1 = figure;
hold on
plot(nodist{1}(:,1),nodist{1}(:,2))
plot(nodist{2}(:,1),nodist{2}(:,2))
plot(nodist{3}(:,1),nodist{3}(:,2))
plot(nodist{4}(:,1),nodist{4}(:,2))
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Open Loop System Response - No Time Delay')
legend('V = 0','V = 12', 'V = 24','V = 48')
saveas(fig1,'No Disturbance Open Loop System Respone.jpg')

for i = 1:length(nodist)
    fig = figure;
    plot(nodist{i}(:,1),nodist{i}(:,2))
    ylabel('Motor Angular Velocity [rad/s]')
    xlabel('Time (s)')
    tstr = sprintf('No Disturbance - Voltage = %.0f V, Fd = %.0f N',volt_vec(i),fd_vec(4));
    sstr = sprintf('NoDist_V%.0f__Fd%.0f.jpg',volt_vec(i),fd_vec(4));
    title(tstr)
    saveas(fig,sstr)
end

%% Open Loop No Delay
nodelay = {};
load_system('openloop_nodelay')
for i = 1:length(volt_vec)
    volt = volt_vec(i);
    fd = fd_vec(4);
    simOut = sim('openloop_nodelay');
    y = simOut.get('ScopeData');
    nodelay = [nodelay y];
end

fig1 = figure;
hold on
plot(nodelay{1}(:,1),nodelay{1}(:,2))
plot(nodelay{2}(:,1),nodelay{2}(:,2))
plot(nodelay{3}(:,1),nodelay{3}(:,2))
plot(nodelay{4}(:,1),nodelay{4}(:,2))
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Open Loop System Response - No Time Delay')
legend('V = 0','V = 12', 'V = 24','V = 48')
saveas(fig1,'No Delay Open Loop System Respone.jpg')

for i = 1:length(nodelay)
    fig = figure;
    plot(nodelay{i}(:,1),nodelay{i}(:,2))
    ylabel('Motor Angular Velocity [rad/s]')
    xlabel('Time (s)')
    tstr = sprintf('No Delay - Voltage = %.0f V, Fd = %.0f N',volt_vec(i),fd_vec(4));
    sstr = sprintf('ND_V%.0f__Fd%.0f.jpg',volt_vec(i),fd_vec(4));
    title(tstr)
    saveas(fig,sstr)
end

%% Open Loop System Response
ystore = {};
load_system('openloop')
for i = 1:length(volt_vec)
    volt = volt_vec(i);
    fd = fd_vec(4);
    time = dist_time(1);
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
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Open Loop System Response')
legend('V = 0','V = 12', 'V = 24','V = 48')
saveas(fig1,'Open Loop System Respone.jpg')

for i = 1:length(ystore)
    fig = figure;
    plot(ystore{i}(:,1),ystore{i}(:,2))
    ylabel('Motor Angular Velocity [rad/s]')
    xlabel('Time (s)')
    tstr = sprintf('Voltage = %.0f V, Fd = %.0f N',volt_vec(i),fd_vec(4));
    sstr = sprintf('V%.0f__Fd%.0f.jpg',volt_vec(i),fd_vec(4));
    title(tstr)
    saveas(fig,sstr)
end

%% Open Loop System Response - Disturbance Change
dchange = {};
load_system('openloop')
for i = 1:length(fd_vec)
    volt = volt_vec(3);
    fd = fd_vec(i);
    time = dist_time(1);
    simOut = sim('openloop');
    y = simOut.get('ScopeData');
    dchange = [dchange y];
end

fig1 = figure;
hold on
plot(dchange{1}(:,1),dchange{1}(:,2))
plot(dchange{2}(:,1),dchange{2}(:,2))
plot(dchange{3}(:,1),dchange{3}(:,2))
plot(dchange{4}(:,1),dchange{4}(:,2))
plot(dchange{5}(:,1),dchange{5}(:,2))
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Open Loop System Response - Disturbance Mangitude Change')
legend('Fd = 0 N','Fd = -196 N','Fd = -275 N','Fd = -353 N','Fd = -432 N')
saveas(fig1,'Dchange_Open Loop System Respone.jpg')

for i = 1:length(dchange)
    fig = figure;
    plot(dchange{i}(:,1),dchange{i}(:,2))
    ylabel('Motor Angular Velocity [rad/s]')
    xlabel('Time (s)')
    tstr = sprintf('Voltage = %.0f V, Fd = %.0f N',volt_vec(3),fd_vec(i));
    sstr = sprintf('DMC_V%.0f__Fd%.0f.jpg',volt_vec(3),fd_vec(i));
    title(tstr)
    saveas(fig,sstr)
end

%% Open Loop System Response - Disturbance Time Change
ystore = {};
load_system('openloop')
for i = 1:length(volt_vec)
    volt = volt_vec(i);
    fd = fd_vec(4);
    time = dist_time(2);
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
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Open Loop System Response - Disturbance at 30s')
legend('V = 0','V = 12', 'V = 24','V = 48')
saveas(fig1,'D30s_Open Loop System Respone.jpg')

for i = 1:length(ystore)
    fig = figure;
    plot(ystore{i}(:,1),ystore{i}(:,2))
    ylabel('Motor Angular Velocity [rad/s]')
    xlabel('Time (s)')
    tstr = sprintf('Voltage = %.0f V, Fd = %.0f N',volt_vec(i),fd_vec(4));
    sstr = sprintf('D30s_V%.0f__Fd%.0f.jpg',volt_vec(i),fd_vec(4));
    title(tstr)
    saveas(fig,sstr)
end

%% Real life
actual = {};
load_system('realsim')

    volt = 12;
    fd = fd_vec(4);
    simOut = sim('realsim');
    y = simOut.get('ScopeData');
    actual = [actual y];


fig1 = figure;
hold on
plot(actual{1}(:,1),actual{1}(:,2))
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Simulated Real Jogging Response')
ylim([60 70])
%legend('V = 0','V = 12', 'V = 24','V = 48')
saveas(fig1,'realsim.jpg')

%% Lead/Lag - Root Locus
close all
sys = timedelay*plant;
figure
rlocus(sys)
title('Root Locus, No Compensation')

t = 0.3111;
a= 0.001404;
b= 0.09233;
c= 0.2121;
d= 0.56;
p = -b/(3*a);
q = p^3+(b*c-3*a*d)/(6*a^2);
r = c/(3*a);
x = (q+(q^2+(r-p^2)^3)^(1/2))^(1/3) + (q-(q^2+(r-p^2)^3)^(1/2))^(1/3) + p;

p = pole(sys);
re_target = -1.31;
im_target = 1.16;

a_choice = 10;
zero = [-a_choice];
poles = [p' re_target];
lenz = length(zero);
lenp = length(poles);
phi = [];
tri = [];

for i = 1:lenz
    if zero(i)>re_target
        dist = abs(re_target-zero(i));
        tricalc = 180-atand(dist/im_target);
        tri = [tri tricalc];
    else
        dist = abs(zero(i)-re_target);
        tricalc = atand(dist/im_target);
        tri = [tri tricalc];
    end
end

for i = 1:lenp-1
    if poles(i)>re_target
        dist = abs(re_target-poles(i));
        phicalc = 180-atand(dist/im_target);
        phi = [phi phicalc];
    else
        dist = abs(poles(i)-re_target);
        phicalc = atand(dist/im_target);
        phi = [phi phicalc];
    end
end

syms x
b_angle = double(solve(sum(tri)-sum(phi)-x==180,x));
b_mod = mod(b_angle,360);
b_pole = im_target/tand(b_mod)+abs(re_target);

s = complex(re_target,im_target);
lead_num = [1 a_choice];
lead_den = [1 b_pole];
lead = tf(lead_num,lead_den);

syms k
gain = double(solve(k*abs(((s+a_choice)/(s+b_pole))*((t)/(a*s^3+b*s^2+c*s+d)))==1,k));

figure
rlocus(gain*lead*sys)
title('Root Locus, with Compensation')
%figure
%step(gain*lead*sys,40);

c = 0.116708;
d = 0.01;
lag_num = [1 c];
lag_den = [1 d];
lag = tf(lag_num,lag_den);

leadlag = gain*lead*lag*sys;

closto = {};
load_system('rootlocus')
for i = 1:length(volt_vec)
    volt = volt_vec(i);
    fd = fd_vec(1);
    time = dist_time(1);
    simOut = sim('rootlocus');
    y = simOut.get('LeadLagScope');
    closto = [closto y];
end

fig_l1 = figure;
hold on
%plot(closto{1}(:,1),closto{1}(:,2))
plot(closto{2}(:,1),closto{2}(:,2))
plot(closto{3}(:,1),closto{3}(:,2))
plot(closto{4}(:,1),closto{4}(:,2))
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Closed Loop Response')
legend('V = 12', 'V = 24','V = 48')


cldchange = {};
load_system('closedloop')
for i = 1:length(fd_vec)
    volt = volt_vec(3);
    fd = fd_vec(i);
    time = dist_time(1);
    simOut = sim('closedloop');
    y = simOut.get('ScopeData1');
    cldchange = [cldchange y];
end

fig12 = figure;
hold on
%plot(lldchange{1}(:,1),lldchange{1}(:,2))
plot(cldchange{2}(:,1),cldchange{2}(:,2))
plot(cldchange{3}(:,1),cldchange{3}(:,2))
plot(cldchange{4}(:,1),cldchange{4}(:,2))
plot(cldchange{5}(:,1),cldchange{5}(:,2))
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Closed Loop System Response - Disturbance Mangitude Change')
legend('Fd = -196 N','Fd = -275 N','Fd = -353 N','Fd = -432 N')
% saveas(fig1,'Dchange_Open Loop System Respone.jpg')



leadlagsto = {};
load_system('rootlocus')
for i = 1:length(volt_vec)
    volt = volt_vec(i);
    fd = fd_vec(1);
    time = dist_time(1);
    simOut = sim('rootlocus');
    y = simOut.get('LeadLagScope');
    leadlagsto = [leadlagsto y];
end

fig_l3 = figure;
hold on
%plot(leadlagsto{1}(:,1),leadlagsto{1}(:,2))
plot(leadlagsto{2}(:,1),leadlagsto{2}(:,2))
plot(leadlagsto{3}(:,1),leadlagsto{3}(:,2))
plot(leadlagsto{4}(:,1),leadlagsto{4}(:,2))
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Lead-Lag Root Locus Closed Loop Response')
legend('V = 12', 'V = 24','V = 48')

lldchange = {};
load_system('rootlocus')
for i = 1:length(fd_vec)
    volt = volt_vec(3);
    fd = fd_vec(i);
    time = dist_time(1);
    simOut = sim('rootlocus');
    y = simOut.get('LeadLagScope');
    lldchange = [lldchange y];
end

fig14 = figure;
hold on
%plot(lldchange{1}(:,1),lldchange{1}(:,2))
plot(lldchange{2}(:,1),lldchange{2}(:,2))
plot(lldchange{3}(:,1),lldchange{3}(:,2))
plot(lldchange{4}(:,1),lldchange{4}(:,2))
plot(lldchange{5}(:,1),lldchange{5}(:,2))
ylabel('Motor Angular Velocity [rad/s]')
xlabel('Time (s)')
title('Compensated System Response - Disturbance Mangitude Change')
legend('Fd = -196 N','Fd = -275 N','Fd = -353 N','Fd = -432 N')
% saveas(fig1,'Dchange_Open Loop System Respone.jpg')

