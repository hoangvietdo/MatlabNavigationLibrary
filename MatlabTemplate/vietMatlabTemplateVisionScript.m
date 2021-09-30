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
path = '';
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

imageFileNames = dir(fullfile(path, '\images\*.jpg')); % raw image
N = length(imageFileNames);

%% Main
for i = 1:1:10
    imgRGB = imread([imageFileNames(i).folder, '\', imageFileNames(i).name]);
    imgGray = rgb2gray(imgRGB);
    fg = imshow(imgGray);
    hold(gca, 'on')
    
    %% Main img processing (Main Work Space)
    
    % plot to children ax
    x = 10; y = 10;
    plot(ax, x, y);
    
    %% Record video
    if makeVideo == 1
        frame = getframe(gcf);
        writeVideo(outputVideo, frame);
    end
end

if makeVideo == 1
    close(outputVideo)
end