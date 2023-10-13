%% Lat/Dir Plant Model
%
% Lateral-Directional Dynamics Plant Model for a transport aircraft in
% cruise configuration from:
% "Robust and Adaptive Control with Aerospace Applications" 
% by Lavretsky and Wise, pg. 25
%
%
% The implemented RSLQR Controller in this file is also sourced from this
% same textbook.

clear;
close all;
%% Plant
% x = [phi, beta, p, r]'
% u = [da, dr]';
% y = [beta, phi]'

% Setup State-Space Names
stateNames = {'Phi [rad]','Beta [rad]', 'p [rps]', 'r [rps]'};
inputNames = {'da [rad]', 'dr [rad]'};
nomOutputNames = {'Phi [rad]','Beta [rad]', 'p [rps]', 'r [rps]'};

% Bare-Airframe, no actuators
A_ba = [ 0.0000,   0.0000,   1.0000,    0.0000;
         0.0487,  -0.0829,   0.0000,   -1.0000;
        -0.0000,  -4.5460,  -1.6990,    0.1717;
         0.0000,   3.3820,  -0.0654,   -0.0893;
];
[nBa, mBa] = size(A_ba);

B_ba = [0.0000,   0.0000;
        0.0000,   0.0116;
        27.276,   0.5758;
        0.3952,  -1.362;
];

C_ba = eye(nBa, mBa);

D_ba = zeros(size(B_ba));


% Nominal System
sys_ba = ss(A_ba, B_ba, C_ba, D_ba, 'StateName', stateNames, 'InputName', inputNames, ...
               'OutputName', nomOutputNames);
nomSys_tf = tf(sys_ba);


%% Add actuators
stateNames = {'da [deg]','dr [deg]'};
inputNames = {'daCmd [deg]', 'drCmd [deg]'};
nomOutputNames = {'da [deg]','dr [deg]'};
wn_act = 20.2;
A_act = [-wn_act,      0;
               0, -wn_act
];
 
B_act = [wn_act,     0;
              0,   wn_act;
];
C_act = [1, 0; 0 1;];
D_act = zeros(size(A_act));
sys_act = ss(A_act, B_act, C_act, D_act,  'StateName', stateNames, 'InputName', inputNames, ...
               'OutputName', nomOutputNames);

sys_actPlusAirframe = series(sys_act,sys_ba);


%% Add tracking error
% e_yi = (y - y_ref)/s
% eDot_yi = C*x + D*u + D_r*r;
% x_aug = [ephi, ebeta, phi, beta, p, r, da, dr]'
augStateNames = {'e_phi', 'e_beta', 'Phi [rad]','Beta [rad]', 'p [rps]', 'r [rps]','da [rad]','dr [rad]'};
augInputNames = {'PhiCmd [rad]','BetaCmd [rad]'};
augOutputNames = {'e_phi', 'e_beta', 'Phi [rad]','Beta [rad]', 'p [rps]', 'r [rps]','da [rad]','dr [rad]'};

A_aug = [zeros(6,2),sys_actPlusAirframe.A];
ephi_row = [0 0 1 0 0 0 0 0];
ebeta_row = [0 0 0 1 0 0 0 0];
A_aug = [ephi_row;ebeta_row;A_aug];


B_aug = [zeros(2,2);sys_actPlusAirframe.B];
C_aug = sys_actPlusAirframe.C;
D_aug = sys_actPlusAirframe.D;


%% Compute LQR Gains

qLqr = diag([10, 10, 0, 0, 0, 0, 0, 0]);
rLqr = eye(2,2);

[kLqr,~,~] = lqr(A_aug, B_aug, qLqr, rLqr);

% Reassign for Simulink Model
ki_beta = kLqr(:,2);
ki_phi = kLqr(:,1);
k_fb = kLqr(:,3:8);



%% Create Closed-loop system
A_cl = A_aug - B_aug*kLqr;
B_cl =[-1*eye(2,2) zeros(2,6)]';
C_cl = eye(size(A_cl));
D_cl = zeros(size(B_cl));

sys_cl = ss(A_cl, B_cl, C_cl, D_cl,'StateName', augStateNames,...
            'InputName', augInputNames, 'OutputName', augOutputNames);


%% Sim it
dt = 0.01;
t = 0:dt:10;
tStep = 2;
step_deg = 20;
u = zeros(length(t),2);
u(tStep/0.01:end,1) = step_deg*pi/180;
[Y,T,X] = lsim(sys_cl, u, t);



%% Make Plots
fn = 1;

% Bare-Airframe poles and zeros
fh = figure(fn);
h_ba = pzplot(sys_ba,'r');
grid on; hold on;
h_cl = pzplot(sys_cl,'b');
title('Bare-Airframe vs Closed-Loop Poles and Zeros');
set(get(fh,'CurrentAxes'),'GridAlpha',0.4,'MinorGridAlpha',0.7);
set(fh, 'Position',[1950 146 1428 948]);
legend('Bare Airframe', 'Closed-Loop');
xlim([-21, 1])
fn = fn+1;

% Zoom in on unstable spiral
fh = figure(fn);
h_ba = pzplot(sys_ba,'r');
grid on; hold on;
h_cl = pzplot(sys_cl,'b');
title('Bare-Airframe Unstable Spiral Mode');
set(get(fh,'CurrentAxes'),'GridAlpha',0.4,'MinorGridAlpha',0.7);
set(fh, 'Position',[1950 146 1428 948]);
legend('Bare Airframe', 'Closed-Loop');
xlim([-.01, .002])
fn = fn+1;

% Actuator Model Step Response
fh = figure(fn);
step(sys_act);
grid on;
set(fh, 'Position',[1950 146 1428 948]);
title('Actuator Model Step Response')
fn = fn+1;


% Closed-loop Command Tracking
fh = figure(fn);
subplot(4,1,1);
plot(T,u(:,1)*180/pi,'LineWidth',2);
grid on; hold on;
plot(T,Y(:,3)*180/pi,'LineWidth',2);
legend('Phi_{Cmd}','Phi');
ylabel('\phi [deg]');
title('LQR-Optimized Lat-Dir Controller');
set(get(fh,'CurrentAxes'),'GridAlpha',0.4,'MinorGridAlpha',0.7);

subplot(4,1,2);
plot(T,u(:,2)*180/pi,'LineWidth',2);
grid on; hold on;
plot(T,Y(:,4)*180/pi,'LineWidth',2);
legend('Beta_{Cmd}','Phi');
ylabel('\beta [deg]');
set(get(fh,'CurrentAxes'),'GridAlpha',0.4,'MinorGridAlpha',0.7);

subplot(4,1,3);
plot(T,Y(:,5)*180/pi,'LineWidth',2);
grid on; hold on;
plot(T,Y(:,6)*180/pi,'LineWidth',2);
legend('p','r');
ylabel('rate [deg/s]');
set(get(fh,'CurrentAxes'),'GridAlpha',0.4,'MinorGridAlpha',0.7);

subplot(4,1,4);
plot(T,X(:,7)*180/pi,'LineWidth',2);
grid on; hold on;
plot(T,X(:,8)*180/pi,'LineWidth',2);
legend('da','dr');
ylabel('\delta [deg]');
xlabel('Time [s]');
set(get(fh,'CurrentAxes'),'GridAlpha',0.4,'MinorGridAlpha',0.7);

set(fh, 'Position',[1950 146 1428 948]);
fn = fn+1;

