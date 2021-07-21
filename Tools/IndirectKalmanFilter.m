%% License: intelligent Navigation and Control System Laboratory (iNCLS) - Sejong University
%  Author : Viet
%  e-Mail : hoangvietdo@sju.ac.kr
%  Date :
%  Method Index : prediction, measurementUpdate, correction

%% TODO
% Add the plus option in correction step
% Add Prediction

%%
classdef IndirectKalmanFilter
    
    % Indirect Kalman Filter for Navigation Systems
    
    properties
    end
    
    methods(Static)
        %-----------------------------------------------------------
        function PPrior = prediction(C, Q, F, dt, PPosterior)
            
            Q(4:6, 4:6) = C * Q(4:6, 4:6) * C';
            Q(4:6, 4:6) = C * Q(4:6, 4:6) * C';
            
            PPrior = F * PPosterior * F' + Q * dt;
            PPrior = (PPrior + PPrior') / 2;

        end
        %-----------------------------------------------------------

        function [xPosterior, PPosterior] = measurementUpdate(H, R, PPrior, z, dt, order)
            
            if nargin < 6
                order = 'PVQ';
            end
            
            R = R / dt;
            K = ( PPrior * H' ) / ( H * PPrior * H' + R );
            xPosterior_ = K * z;
            
            switch(order)
                case 'PVQ'
                    xPosterior.Position = xPosterior_(1:3);
                    xPosterior.Velocity = xPosterior_(4:6);
                    xPosterior.Euler = xPosterior_(7:9);
                    xPosterior.biasAccel = xPosterior_(10:12);
                    xPosterior.biasGyro = xPosterior_(13:15);
                case 'QVP'
                    xPosterior.Position = xPosterior_(7:9);
                    xPosterior.Velocity = xPosterior_(4:6);
                    xPosterior.Euler = xPosterior_(1:3);
                    xPosterior.biasAccel = xPosterior_(10:12);
                    xPosterior.biasGyro = xPosterior_(13:15);
            end
            
            I = eye(size(PPrior, 1));
            PPosterior = (I - K * H) * PPrior * (I - K * H)' + K * R * K';
            PPosterior = (PPosterior + PPosterior') / 2;
        end
        %-----------------------------------------------------------
        
        function updateState = correction(currentState, deltaState, operator)
            if nargin < 6
                operator = 'minus';
            end
            
            updateState = currentState;
            
            switch(operator)
                case 'minus'
                    updateState.Position = currentState.Position - deltaState.Position;
                    updateState.Velocity = currentState.Velocity - deltaState.Velocity;
                    updateState.Quaternion = Attitude.quatCorrectionIKF(currentState.Quaternion, deltaState.Euler); 
                    updateState.biasAccel = currentState.biasAccel + deltaState.biasAccel;
                    updateState.biasGyro = currentState.biasGyro + deltaState.biasGyro;
                    updateState.R = Attitude.quat2dcm(updateState.Quaternion);
                    updateState.Euler = Attitude.quat2euler(updateState.Quaternion);
                case 'plus'
            end
        end
    end
    
end
