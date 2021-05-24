%% V iNCSL

%% TODO

%% Notation for the code

%% Clean Stuff
clear;
close all
clc;

%% Add Path
addpath('Tools');
addpath('Datasets');

%% Load Dataset
% load('car_tj_noise.mat')
% load('car_trj_12m.mat')
load('VINS_measurement.mat')

%% Define Parameters for simulation
g = 9.8;
gVec = [0; 0; g];
d2r = pi / 180;
r2d = 180 / pi;
mg = 1e-3 * g;
dPrh = 1 / 60 * d2r ;            % deg/sqrt(hr)
dPh = 1 / 3600 * d2r;            % deg/hr
Tsim = 10;
dTimu = 1/100; Nimu = Tsim/dTimu;
dTcam = 1/20; Ncam = Tsim/dTcam;
intOpt = '2ndOrder';
flag = 'meter';
randomWalk = -0.0001;

%% Extract data from dataset
% gT.Time = time;
% gT.Euler = attitude;
% gT.Velocity = vel_true_NED;
% gT.Position = pos_true_NED;

epV = zeros(3, length(gT.Time));
epP = zeros(3, length(gT.Time));
epA = zeros(3, length(gT.Time));

%%
R0 = Attitude.euler2dcm(gT.Euler(:, 1));
p0 = gT.Position(:, 1);
v0 = gT.Velocity(:, 1);
state = INS.buildState(p0, v0, R0, zeros(3,1), zeros(3,1), gVec);
oldState = zeros(3, 2);
rate_noise = zeros(3, length(gT.Time));
accel_noise = zeros(3, length(gT.Time));
dt = 0.01;

for i = 1:1:length(gT.Time)
    rate_noise(:, i) = measurementCleaned.IMU{i}.Gyro;
    accel_noise(:, i) =  measurementCleaned.IMU{i}.Accel;
    meas = [rate_noise(:, i) , accel_noise(:, i)];

    [state, preState, Fk] = INS.localNav(state, oldState, meas, dt, intOpt, flag, i, randomWalk);
    oldState = preState;
    %     ep(i) = rms((state.Velocity(3) - groundTruth.Velocity(3, i)), 2);
    %     foo = Attitude.dcm2euler(state.R);
    fooV = state.Velocity;
    epV(:, i) = fooV;
    fooP = state.Position;
    epP(:, i) = fooP;
    fooA = Attitude.dcm2euler(state.R);
    epA(:, i) = fooA;
end

%%
% close all;

for idx = 1:1:3
    A = cumsum(accel_noise(idx, :)*0.01);
    B = cumsum(rate_noise(idx, :)*0.01);
    C = cumsum(epV(idx, :)*0.01);
    
    epc = A;
    figure(3 * (idx - 1) + 1),
    plot(gT.Time, epV(idx, :),'r'); hold on; grid on;
    plot(gT.Time, epc,'b');
    plot(gT.Time, gT.Velocity(idx, :),'g');
    legend('INS', 'Integra','GT');

    epc = B;    
    figure(3 * (idx - 1) + 2),
    plot(gT.Time, epA(idx, :)*r2d,'r'); hold on; grid on;
    plot(gT.Time, epc*r2d,'b');
    plot(gT.Time, gT.Euler(idx, :)*r2d,'g');
    legend('INS', 'Integra','GT');
    
    epc = C;
    figure(3 * (idx - 1) + 3),
    plot(gT.Time, epP(idx, :),'r'); hold on; grid on;
%     plot(gT.Time, epc,'b');
    plot(gT.Time, gT.Position(idx, :),'g');
%     legend('INS', 'Integra','GT');
    legend('INS','GT');
end

figure(100),
plot(epP(1,:), epP(2,:))
