%% License: intelligent Navigation and Control System Laboratory (iNCLS) - Sejong University
%  Author : Viet
%  e-Mail : hoangvietdo@sju.ac.kr
%  Date :
%  Method Index : splitStructure, drawEstimation, drawSetting, drawRMSE, drawErrorBound

%% TODO
% Add the plus option in correction step

%%
classdef vPlotting
    
    %
    
    properties(Constant)
        color1 = [55,126,184]/255;
        color2 = [1,0.6,0.78];
        color3 =  [255,0,255]/255;
        colorOrders = [0 0 0; 0.4, 0.4, 0.4; 0.7, 0.7, 0.7];
        d2r = pi / 180;
        r2d = 180 / pi;
    end
    
    methods(Static)
        %-----------------------------------------------------------
        function self = splitStructure(x, varName)
            N = max(size(x, 1), size(x, 2));
            M = length(x{1}.(varName));
            self = zeros(M, N);
            for i = 1:1:N
                self(:, i) = x{i}.(varName);
            end
        end
        %-----------------------------------------------------------
        
        function drawEstimate(gT, Est, flag)
            
            %% Check Flag
            if nargin < 3
                flag = 0;
            end
            
            %% Position
            figure('NumberTitle', 'off', 'Name', 'Position Estimation');
            %
            subplot(311);
            plot(gT.Position(1, :)); hold on;
            plot(Est.Position(1, :)); hold off; ylabel('X [m]');
            xlim([0 max(length(gT.Position), length(Est.Position))]);
            title('Position Estimation');
            legend('True','Estimate', 'Location', 'Best', 'Interpreter', 'LaTex');
            %
            subplot(312);
            plot(gT.Position(2, :)); hold on;
            plot(Est.Position(2, :)); hold off; ylabel('Y [m]');
            xlim([0 max(length(gT.Position), length(Est.Position))]);
            %
            subplot(313);
            plot(gT.Position(3, :)); hold on;
            plot(Est.Position(3, :)); hold off; ylabel('Z [m]');
            xlabel('Time [sec]');
            xlim([0 max(length(gT.Position), length(Est.Position))]);
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/est_position.png', 'Resolution', 600);
            end
            
            %% Velocity
            figure('NumberTitle', 'off', 'Name', 'Velocity Estimation');
            %
            subplot(311);
            plot(gT.Velocity(1, :)); hold on;
            plot(Est.Velocity(1, :)); hold off; ylabel('X [m/s]');
            xlim([0 max(length(gT.Velocity), length(Est.Velocity))]);
            title('Velocity Estimation');
            legend('True', 'Estimate', 'Location', 'Best', 'Interpreter', 'LaTex');
            
            %
            subplot(312);
            plot(gT.Velocity(2, :)); hold on;
            plot(Est.Velocity(2, :)); hold off; ylabel('Y [m/s]');
            xlim([0 max(length(gT.Velocity), length(Est.Velocity))]);
            %
            subplot(313);
            plot(gT.Velocity(3, :)); hold on;
            plot(Est.Velocity(3, :)); hold off; ylabel('Z [m/s]');
            xlabel('Time [sec]');
            xlim([0 max(length(gT.Velocity), length(Est.Velocity))]);
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/est_velocity.png', 'Resolution', 600);
            end
            
            %% Attitude
            figure('NumberTitle', 'off', 'Name', 'Attitude Estimation');
            subplot(311);
            plot(gT.Euler(1, :)); hold on;
            plot(Est.Euler(1, :) * vPlotting.r2d); hold off; ylabel('Roll [Deg]');
            xlim([0 max(length(gT.Euler), length(Est.Euler))]);
            
            title('Attitude Estimation');legend('True','Estimate', 'Location', 'Best', 'Interpreter', 'LaTex');
            
            subplot(312);
            plot(gT.Euler(2, :)); hold on;
            plot(Est.Euler(2, :) * vPlotting.r2d); hold off; ylabel('Pitch [Deg]');
            xlim([0 max(length(gT.Euler), length(Est.Euler))]);
            
            subplot(313);
            plot(gT.Euler(3, :)); hold on;
            plot(Est.Euler(3, :) * vPlotting.r2d); hold off; ylabel('Yaw [Deg]');
            xlim([0 max(length(gT.Euler), length(Est.Euler))]);
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/est_attitude.png', 'Resolution', 600);
            end
            
            %% Accel Bias
            figure('NumberTitle', 'off', 'Name', 'Accelerometer Bias Estimation');
            subplot(311); plot(Est.biasAccel(1, :)); ylabel('ba_x [m/s^2]');
            xlim([0 max(length(Est.biasAccel))]);
            
            title('Accel Bias Estimation');legend('Estimate', 'Location', 'Best', 'Interpreter', 'LaTex');
            subplot(312); plot(Est.biasAccel(2, :)); ylabel('ba_y [m/s^2]');
            xlim([0 max(length(Est.biasAccel))]);
            
            subplot(313); plot(Est.biasAccel(3, :)); ylabel('ba_z [m/s^2]');
            xlim([0 max(length(Est.biasAccel))]);
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/est_biasAccel.png', 'Resolution', 600);
            end
            
            %% Gyro Bias
            figure('NumberTitle', 'off', 'Name', 'Gyroscope Bias Estimation');
            subplot(311); plot(Est.biasGyro(1, :) * vPlotting.r2d); ylabel('bg_x [deg/s]');
            xlim([0 max(length(Est.biasGyro))]);
            title('Gyro Bias Estimation');legend('Estimate', 'Location', 'Best', 'Interpreter', 'LaTex');
            
            subplot(312); plot(Est.biasGyro(2, :) * vPlotting.r2d); ylabel('bg_y [deg/s]');
            xlim([0 max(length(Est.biasGyro))]);
            
            subplot(313); plot(Est.biasGyro(3, :) * vPlotting.r2d); ylabel('bg_z [deg/s]');
            xlim([0 max(length(Est.biasGyro))]);
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/est_biasGyro.png', 'Resolution', 600);
            end
        end
        %-----------------------------------------------------------
        
        function drawRMSE(rmse, flag)
            
            %% Check Flag
            if nargin < 2
                flag = 0;
            end
            
            %% Position
            figure('NumberTitle', 'off', 'Name', 'RMSE of Position Estimation');
            subplot(311);plot(rmse.Position(1, :)); ylabel('X [m]');
            xlim([0 max(length(rmse.Position))]);
            title('Root Mean Square Error of Position Estimation');
            
            subplot(312);plot(rmse.Position(2, :)); ylabel('Y [m]');
            xlim([0 max(length(rmse.Position))]);
            
            subplot(313);plot(rmse.Position(3, :)); ylabel('Z [m]');
            xlim([0 max(length(rmse.Position))]);
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/rmse_position.png', 'Resolution', 600);
            end
            
            %% Velocity
            figure('NumberTitle', 'off', 'Name', 'RMSE of Velocity Estimation');
            subplot(311);plot(rmse.Velocity(1, :)); ylabel('X [m/s]');
            xlim([0 max(length(rmse.Velocity))]);
            title('Root Mean Square Error of Velocity Estimation');
            
            subplot(312);plot(rmse.Velocity(2, :)); ylabel('Y [m/s]');
            xlim([0 max(length(rmse.Velocity))]);
            
            subplot(313);plot(rmse.Velocity(3, :)); ylabel('Z [m/s]');
            xlim([0 max(length(rmse.Velocity))]);
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/rmse_velocity.png', 'Resolution', 600);
            end
            
            %% Attitude
            figure('NumberTitle', 'off', 'Name', 'RMSE of Attitude Estimation');
            subplot(311);plot(rmse.Euler(1, :) * vPlotting.r2d); ylabel('X [Deg]');
            xlim([0 max(length(rmse.Euler))]);
            title('Root Mean Square Error of Attitude Estimation');
            
            subplot(312);plot(rmse.Euler(2, :) * vPlotting.r2d); ylabel('Y [Deg]');
            xlim([0 max(length(rmse.Euler))]);
            
            subplot(313);plot(rmse.Euler(3, :) * vPlotting.r2d); ylabel('Z [Deg]');
            xlim([0 max(length(rmse.Euler))]);
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/rmse_euler.png', 'Resolution', 600);
            end
        end
        %-----------------------------------------------------------
        
        function drawErrorBound(error, P, flag)
            
            %% Check Flag
            if nargin < 3
                flag = 0;
            end
            
            var = sqrt(P);
            
            %% Position
            figure('NumberTitle', 'off', 'Name', 'Error Bound of Position');
            subplot(311);plot(error(1, :)); hold on;
            plot(3 * var(1, :), 'r');
            plot(-3 * var(1, :), 'r'); hold off; ylabel('X [m]');
            title('Estimation Error vs. 3\sigma bounds of Position');
            
            subplot(312);plot(error(2, :)); hold on;
            plot(3 * var(2, :), 'r')
            plot(-3 * var(2, :), 'r'); hold off; ylabel('Y [m]');
            subplot(313);plot(error(3, :)); hold on;
            plot(3 * var(3, :), 'r')
            plot(-3 * var(3, :), 'r'); hold off; ylabel('Z [m]');
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/eb_position.png', 'Resolution', 600);
            end
            
            %% Velocity
            figure('NumberTitle', 'off', 'Name', 'Error Bound of Velocity');
            subplot(311);plot(error(4, :)); hold on;
            plot(3 * var(4, :), 'r')
            plot(-3 * var(4, :), 'r'); hold off; ylabel('X [m/s]');
            title('Estimation Error vs. 3\sigma bounds of Velocity');
            subplot(312);plot(error(5, :)); hold on;
            plot(3 * var(5, :), 'r')
            plot(-3 * var(5, :), 'r'); hold off; ylabel('Y [m/s]');
            subplot(313);plot(error(6, :)); hold on;
            plot(3 * var(6, :), 'r')
            plot(-3 * var(6, :), 'r'); hold off; ylabel('Z [m/s]');
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/eb_velocity.png', 'Resolution', 600);
            end
            %% Attitude
            figure('NumberTitle', 'off', 'Name', 'Error Bound of Attitude');
            subplot(311);plot(error(7, :) * vPlotting.r2d); hold on;
            plot(3 * var(7, :) * vPlotting.r2d, 'r')
            plot(-3 * var(7, :) * vPlotting.vPlotting.r2d, 'r'); hold off; ylabel('X [Deg]');
            title('Estimation Error vs. 3\sigma bounds of Attitude');
            subplot(312);plot(error(8, :) * vPlotting.r2d); hold on;
            plot(3 * var(8, :) * vPlotting.r2d, 'r')
            plot(-3 * var(8, :) * vPlotting.r2d, 'r'); hold off; ylabel('Y [Deg]');
            subplot(313);plot(error(9, :) * vPlotting.r2d); hold on;
            plot(3 * var(9, :) * vPlotting.r2d, 'r')
            plot(-3 * var(9, :) * vPlotting.r2d, 'r'); hold off; ylabel('Z [Deg]');
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/eb_attitude.png', 'Resolution', 600);
            end
            %% Bias Estimation
            figure('NumberTitle', 'off', 'Name', 'Error Bound of Accel Bias');
            subplot(311);plot(error(10, :)); hold on;
            plot(3 * var(10, :), 'r')
            plot(-3 * var(10, :), 'r'); hold off; ylabel('X [mg]');
            title('Estimation Error vs. 3\sigma bounds of Accelerometer Bias');
            subplot(312);plot(error(11, :)); hold on;
            plot(3 * var(11, :), 'r')
            plot(-3 * var(11, :), 'r'); hold off; ylabel('Y [mg]');
            subplot(313);plot(error(12, :)); hold on;
            plot(3 * var(12, :), 'r')
            plot(-3 * var(12, :), 'r'); hold off; ylabel('Z [mg]');
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/eb_accelBias.png', 'Resolution', 600);
            end
            %% Gyro Estimation
            figure('NumberTitle', 'off', 'Name', 'Error Bound of Gyro Bias');
            subplot(311);plot(error(13, :) * vPlotting.r2d); hold on;
            plot(3 * var(13, :) * vPlotting.r2d, 'r')
            plot(-3 * var(13, :) * vPlotting.r2d, 'r'); hold off; ylabel('X [Deg/s]');
            title('Estimation Error vs. 3\sigma bounds of Gyroscope Bias');
            subplot(312);plot(error(14, :) * vPlotting.r2d); hold on;
            plot(3 * var(14, :) * vPlotting.r2d, 'r')
            plot(-3 * var(14, :) * vPlotting.r2d, 'r'); hold off; ylabel('Y [Deg/s]');
            subplot(313);plot(error(15, :) * vPlotting.r2d); hold on;
            plot(3 * var(15, :) * vPlotting.r2d, 'r')
            plot(-3 * var(15, :) * vPlotting.r2d, 'r'); hold off; ylabel('Z [Deg/s]');
            xlabel('Time [sec]');
            
            if flag == 1
                set(gcf, 'PaperSize', [6 4.5]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 4.5]);
                exportgraphics(gcf, 'Figures/eb_gyroBias.png', 'Resolution', 600);
            end
        end
        %-----------------------------------------------------------
        
        function drawTrajectory(gT, Est, flag)
            
            %% Check Flag
            if nargin < 3
                flag = 0;
            end
            
            %% 2D
            switch(dimension)
                case 2
                    figure('NumberTitle', 'off', 'Name', '2D Trajectory Estimation');
                    plot(gT.Position(1, :), gT.Position(2, :), 'b'); hold on;
                    plot(Est.Position(1, :), Est.Position(2, :), 'r');
                    plot(gT.Position(1, 1), gT.Position(2, 1), 'k*', 'MarkerFaceColor', 'k');
                    plot(gT.Position(1, end), gT.Position(2, end), 'ks', 'MarkerFaceColor', 'k'); hold off;
                    title('2D Trajectory Estimation');
                    xlabel('X [m]');
                    ylabel('Y [m]');
                    legend('Ground Truth', 'Estimation', 'Start', 'End');
                    axis equal;
                    
                    if flag == 1
                        set(gcf, 'PaperSize', [6 4.5]);
                        set(gcf, 'PaperPositionMode', 'manual');
                        set(gcf, 'PaperPosition', [0 0 6 4.5]);
                        exportgraphics(gcf, 'Figures/2Dtrajectory.png', 'Resolution', 600);
                    end
                    
                case 3
                    figure('NumberTitle', 'off', 'Name', '3D Trajectory Estimation');
                    plot3(gT.Position(1, :), gT.Position(2, :), gT.Position(3, :), 'b'); hold on;
                    plot3(Est.Position(1, :), Est.Position(2, :), Est.Position(3, :), 'r');
                    plot3(gT.Position(1, 1), gT.Position(2, 1), gT.Position(3, 1), 'k*', 'MarkerFaceColor', 'k');
                    plot3(gT.Position(1, end), gT.Position(2, end), gT.Position(3, end), 'ks', 'MarkerFaceColor', 'k'); hold off;
                    title('3D Trajectory Estimation');
                    xlabel('X [m]');
                    ylabel('Y [m]');
                    zlabel('Z [m]');
                    legend('Ground Truth', 'Estimation', 'Start', 'End');
                    axis equal;
                    
                    if flag == 1
                        set(gcf, 'PaperSize', [6 4.5]);
                        set(gcf, 'PaperPositionMode', 'manual');
                        set(gcf, 'PaperPosition', [0 0 6 4.5]);
                        exportgraphics(gcf, 'Figures/3Dtrajectory.png', 'Resolution', 600);
                    end
            end
        end
        %-----------------------------------------------------------
        
        function drawRootSetting()
            if nargin == 1
                reset(groot);
            else
                set(groot, 'DefaultAxesXGrid', 'on')
                set(groot, 'DefaultAxesYGrid', 'on')
                set(groot, 'DefaultAxesTickLabelInterpreter','LaTex');
                set(groot, 'DefaultTextInterpreter', 'LaTex')
                set(groot, 'DefaultLineLineWidth', 1.5);
                set(groot, 'DefaultAxesFontSize', 13);
                set(groot, 'DefaultAxesFontWeight', 'Bold');
                %                 set(groot, 'DefaultAxesColorOrder', vPlotting.colorOrders)
                set(groot, 'DefaultAxesLineStyleOrder', '-|:|.-')
            end
        end
        %-----------------------------------------------------------
    end
end