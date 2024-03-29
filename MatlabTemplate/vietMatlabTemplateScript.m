%% License: intelligent Navigation and Control System Laboratory (iNCSL) - Sejong University
%  Author : Viet
%  e-Mail : hoangvietdo@sju.ac.kr
%  Date :

%% TODO

%% Notation for the code

%% Clear and Close
clear;
close all;
clc;

%% Add path
addpath('Tools');
addpath('Datasets');
addpath('C:\Users\Viet Do\OneDrive\Documents\MatlabCode\MatlabNavigationLibrary\Tools');
vPlotting.drawRootSetting(0);

%% Simulation Paramerter
makeVideo = 0; % 0 = No, 1 == Yes;

fg = figure(100);
ax = axes('Parent', fg);

if makeVideo == 1
    set(fg, 'Visible', 'off');
    outputVideo = VideoWriter([path, 'main.avi']);
    outputVideo.FrameRate = 30;
    open(outputVideo);
else
    set(fg, 'Visible', 'on');
end

%% Main
for i = 1:1:10
    if makeVideo == 1
        frame = getframe(gcf);
        writeVideo(outputVideo, frame);
    end
end

if makeVideo == 1
    close(outputVideo)
end