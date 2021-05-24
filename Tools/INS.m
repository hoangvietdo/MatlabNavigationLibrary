 %% License: intelligent Navigation and Control System Laboratory (iNCLS) - Sejong University
%  Author : Viet
%  e-Mail : hoangvietdo@sju.ac.kr
%  Method Index : localNav , buildState

%% TODO
% Add gravity model  ~ Line 179
% Add 4th Runge Kutta Integration ~ integralOpt ~ Line 140

%%
classdef INS
    
    % This class describes a comprehensive INS mechanization in many different coordinate frame
    % such as: ECI, ECEF, Local Navigation NED, ...
    
    % Math Equations and Notations used in this class are mainly from:
    % Reference:  Titterton, D. and J. Weston, STRAPDOWN
    %                   INERTIAL NAVIGATION TECHNOLOGY, Peter
    %                   Peregrinus Ltd. on behalf of the Institution
    %                   of Electrical Engineers, London, 1997.
    
    properties(Constant)
        w_e = [0; 0; 7.292115e-5]; % Earth rate with respect to the {E} frame
        R0 = 6.369121969018592e+06; % Radius of earth
    end
    
    properties
        Position;		% position
        Velocity;		% velocity
        R;		% R^g_b - Rotational Matrix Transform vector from body frame to global frame (ECEF, NED,...)
        Quaternion;		% q^g_b - Quaternion Transform vector from body frame to global frame (ECEF, NED,...)
        biasAccel;		% bias of accelerometer
        biasGyro;		% bias of gyro
        gVec;		% gravity at {G}
        % 		phi;	% ... 
    end
    
    methods(Static)
        function [updateState, prevState, Fk] = localNav(state, oldState, measurement, dt, integralOpt, flag, timeStamp,randomWalk,order)            
            
            % Local Navigation frame - NED INS mechanization
            % Ref 1 - Page 32 + 47 + 342
            %-----------------------------------------------------------
            
            % Notation
            % Xab means X^a_b = X of b with respect to a
            % n = navigation frame
            % b = body frame
            % R = Rotational Matrix
            % q = quaternion
            % v = velocity
            % p = position
            % w = angular rate
            % f = specific force
            % gVec = gravitational force [3x1] Vector with respect to global frame {G}
            %-----------------------------------------------------------
            
            % Option Inputs:
            % Flag = rad will activate the calculator for wn_in
            % Flag = meter will ignore the term wn_in ~ wn_in = 0
            %-----------------------------------------------------------
            
            % Check the flag
            if(isempty(flag))
                flag = 'meter';
            end
            
            % Check order
            if(isempty(order))
                order = 'PVQ';
            end
            
            % Check the integralOpt
            if(isempty(integralOpt))
                integralOpt = '2ndOrder';
            end
            
            % Set up variable
            wb = measurement(:, 1) - state.biasGyro; % measurement from gyroscope
            fb = measurement(:, 2) - state.biasAccel; % measurement from acceleration
            
            biasGyro = state.biasGyro;
            biasAccel = state.biasAccel;
            Rnb = state.R;
            qnb = state.Quaternion;
            vnb = state.Velocity;
            pnb = state.Position;
            gVec = state.gVec;
            
            % Initialization
            updatePnb = zeros(3, 1);
            updateVnb = zeros(3, 1);
            
            % Attitude Update using Quaternion
            switch(flag)
                case 'rad'
                    w_e = INS.w_e;
                    [Rn, Re] = radiusWGS84(pnb(1));
                    wn_ie = [w_e*cos(pnb(1)), 0, -w_e*sin(pnb(1))]'; % Eq. 3.72
                    wn_en = [vnb(2)/(Re + pnb(3)), -vnb(1)/(Rn + pnb(3)), -vnb(2)*tan(pnb(1))/(Re + pnb(3))]'; % Eq. 3.87
                
                case 'meter'
                    wn_ie = zeros(3, 1);
                    wn_en = zeros(3, 1);
            end
            
            wn_in = wn_ie + wn_en;
            wb_in = Rnb' * wn_in;
            
            wb_nb = wb - wb_in;
            
            sigma = wb_nb * dt;
            rk = Attitude.rvec2quat(sigma);
            updateQnb = Attitude.quatMultiply(qnb, rk);
            
%             temp = Attitude.quat2euler(updateQnb);
%             temp_ = temp;
%             temp_(1) = -temp(1);
%             temp_(2) = -temp(2);
%             temp_(3) = -temp(3);
%             updateRnb = Attitude.euler2dcm(temp_);

            updateRnb = Attitude.quat2dcm(updateQnb);
            
            % Velocity + Position Update
            omegaWn_en = vSO3.skewMatrix(wn_en);
            omegaWn_ie = vSO3.skewMatrix(wn_ie);
%             vnbDotPrev = oldState(:, 1);
%             pnbDotPrev = oldState(:, 2);
            
            switch(integralOpt)
                case '2ndOrder' % Trapezoidal Integration
                    % Velocity
%                     fn = 1/2 * (Rnb + updateRnb) * fb;
                    fn = updateRnb * fb;
                    vnbDot = fn - (omegaWn_en + 2 * omegaWn_ie) * vnb + gVec;
                    if timeStamp == 1
                        vnbDotPrev = vnbDot;
                    else
                        vnbDotPrev = oldState(:, 1);
                    end
                    updateVnb = vnb + 1/2 * (vnbDot + vnbDotPrev) * dt;
                    vnbDotPrev = vnbDot;
                    
                    % Position
                    switch(flag)
                        case 'rad'
                            pnbDot(1) = updateVnb(1) / (Rn + pnb(3)); % Eq. 3.79
                            pnbDot(3) = -updateVnb(3); % Eq. 3.81
                            
                            if timeStamp == 1
                                pnbDotPrev(3) = pnbDot(3);
                                pnbDotPrev(1) = pnbDot(1);
                            else
                                pnbDotPrev = oldState(:, 2);
                            end
                            
                            updatePnb(3) = pnb(3) + 1/2 * (pnbDot(3) +  (3)) * dt;
                            updatePnb(1) = pnb(1) + 1/2 * (pnbDot(1) + pnbDotPrev(1)) * dt;
                            
                            [~, Re] = INS.radiusWGS84(updatePnb(1));
                            pnbDot(2) = updateVnb(2) / ((Re + pnb(3)) * cos(pnb(1)));
                            
                            if timeStamp == 1
                                pnbDotPrev(2) = pnbDot(2);
                            end
                                
                            updatePnb(2) = pnb(2) + 1/2 * (pnbDot(2) + pnbDotPrev(2)) * dt;
                            pnbDotPrev = pnbDot';
                        
                        case 'meter'
                            pnbDot = updateVnb;
                            
                            if timeStamp == 1
                                pnbDotPrev = pnbDot;
                            
                            else
                                pnbDotPrev = oldState(:, 2);
                            end
                            
                            updatePnb = pnb + 1/2 * (pnbDot + pnbDotPrev) * dt;
                            pnbDotPrev = pnbDot;
                    end
                case '4thOrder' % 4th Order Runge Kutta 
                    % ~ Working...

                otherwise
                    error('Not Proper Integration Option')
            end
            
            % Compose Update State + preState
            updateState = INS.buildState(updatePnb, updateVnb, updateQnb, biasAccel, biasGyro, gVec);
            prevState = [vnbDotPrev, pnbDotPrev];
            
            % Calculate the F matrix for IMU error model
            Z = zeros(3,3);
            
            switch(flag)
                case 'rad'
                    v_n = updateVnb(1);
                    v_e = updateVnb(2);
                    v_d = updateVnb(3);
                    lat = updatePnb(1);
                    h = updatePnb(3);
                    M = Rn;
                    N = Re;
                    w_e = INS.w_e;
                    
                    % Position error
                    F11 = [                      0                             ,   0   ,   -v_n/((M+h)^2);...
                                v_e*sin(lat)/((N+h)*cos(lat)^2)   ,   0   ,   -v_e/((N+h)^2*cos(lat));...
                                                     0                             ,   0   ,                   0                    ];
                    
                    F12 = [ 1/(M+h)     ,                 0                ,    0;...
                                      0          ,    1/((N+h)*cos(lat))   ,    0;...
                                      0          ,                  0                ,   -1];
                    
                    F13 = Z;
                    F14 = Z;
                    F15 = Z;
                    
                    % Error Velocity
                    %                     foo = 2*gamma/(R+h);
                    foo = 0;
                    F21 = [     -2*v_e*w_e*cos(lat) - v_e^2/((N+h)*cos(lat)^2)                             ,   0   ,       -v_n*v_d/((M+h)^2) + v_e^2*tan(lat)/((N+h)^2);...
                                    2*w_e*(v_n*cos(lat) - v_d*sin(lat)) + v_e*v_n/((N+h)*cos(lat)^2)   ,   0   ,       -v_e*v_d/((N+h)^2) - v_n*tan(lat)/((N+h)^2);...
                                                 2*v_e*w_e*sin(lat)                                                                 ,   0   ,        v_e^2/((N+h)^2) + v_n^2/((M+h)^2) - foo ];
                    
                    F22 = [                 v_d/(M+h)                               ,  -2*w_e*sin(lat) - 2*v_e*tan(lat)/(N+h)    ,   v_n/(M+h);...
                                    2*w_e*sin(lat) + v_e*tan(lat)/(N+h)   ,          (v_d + v_n*tan(lat))/(N+h)               ,   2*w_e*cos(lat) + v_e/(N+h);...
                                            -2*v_n/(M+h)                             ,           -2*w_e*cos(lat)+2*v_e/(N+h)        ,             0];
                    F23 = vSO3.skewMatrix(fn);
                    F24 = Rnb;
                    F25 = Z;
                    
                    % Error Attitude
                    
                    F31 = [                     -w_e*sin(lat)                        ,    0   ,     -v_e/((N+h)^2);...
                                                              0                                 ,    0   ,      v_n/((M+h)^2);...
                                -w_e*cos(lat) - v_e/((N+h)*cos(lat)^2)   ,    0   ,      v_e*tan(lat)/((N+h)^2)];
                    
                    F32 = [             0          ,      1/(N+h)         ,     0;...
                                    -1/(M+h)      ,            0              ,     0;...
                                            0          ,  -tan(lat)/(N+h)   ,     0];
                    
                    F33 = vSO3.skewMatrix([-w_e*cos(lat)-v_e/(N+h)  , v_n/(M+h) , w_e*sin(lat)+v_e*tan(lat)/(N+h)]');
                    F34 = Z;
                    F35 = -Rnb;
                case 'meter'
                    switch(order)
                        case 'PVQ'
                            % P V Q
                            F11 = Z;
                            F12 = eye(3);
                            F13 = Z;
                            F14 = Z;
                            F15 = Z;
                    
                            F21 = Z;
                            F22 = Z;
                            F23 = vSO3.skewMatrix(fn);
                            F24 = Rnb;
                            F25 = Z;
                    
                            F31 = Z;
                            F32 = Z;
                            F33 = Z;
                            F34 = Z;
                            F35 = -Rnb;
                        case 'QVP'
                            % Q V P
                            F11 = Z;
                            F12 = Z;
                            F13 = Z;
                            F14 = Z;
                            F15 = -Rnb;
                    
                            F21 = Z;
                            F22 = Z;
                            F23 = vSO3.skewMatrix(fn);
                            F24 = Rnb;
                            F25 = Z;
                    
                            F31 = Z;
                            F32 = eye(3);
                            F33 = Z;
                            F34 = Z;
                            F35 = Z;
                    end
            end
            % Accelerator and Gyroscope Bias
            % Accel Bias
            F41 = Z ; F42 = Z ; F43 = Z ; F44 = randomWalk * eye(3) ; F45 = Z ;
            
            % Gyro Bias
            F51 = Z ; F52 = Z ; F53 = Z ; F54 = Z ; F55 = randomWalk * eye(3) ;
            
            F = [ F11 , F12 ,  F13 ,  F14 ,  F15;
                F21  , F22 ,  F23 ,  F24 ,  F25;
                F31  , F32 ,  F33 ,  F34 ,  F35;
                F41  , F42 ,  F43 ,  F44 ,  F45;
                F51  , F52 ,  F53 ,  F54 ,  F55];
            
            Fk = eye(15) + F*dt;
        end
        %-----------------------------------------------------------
        
        function [Rn, Re] = radiusWGS84(lat)
            
            % WGS84 parameters setting
            % Ref 1 - Page 49
            R = 6378137.0;				% Semi-Major axis
            r = 6356752.3142;		 % Semi-Minor axis
            f = (R - r) / R;                % Flattening
            e = sqrt(f * (2 - f));           % Major eccentricity
            
            sL = sin(lat);
            Rn = R * (1 - e^2) / (sqrt(1.0 - e^2 * sL^2))^3;	% Meridian Radius(R_N)
            Re = R / (sqrt(1.0 - e^2 * sL^2));                     % Prinme Radius(R_E)
        end
        %-----------------------------------------------------------
        
        function obj = buildState(p_, v_, att, ba_, bg_, g_)
            validateattributes(p_,{'double'},{'size', [3,1]})
            validateattributes(v_,{'double'},{'size', [3,1]})
            validateattributes(ba_,{'double'},{'size', [3,1]})
            validateattributes(bg_,{'double'},{'size', [3,1]})
            validateattributes(g_,{'double'},{'size', [3,1]})
            
            if(isequal(size(att),[3,3]))
            	obj.R = att;
            	obj.Quaternion = Attitude.dcm2quat(att);
                obj.Euler = Attitude.dcm2euler(att);
%                 obj.Euler = vSO3.logmap(att);

            elseif(isequal(size(att),[4,1]))
            	obj.Quaternion = att;
            	obj.R = Attitude.quat2dcm(att);
                obj.Euler = Attitude.quat2euler(att);
%                 obj.Euler = vSO3.logmap(obj.R);

%             elseif(isequal(size(att),[3,1]))
%             	obj.phi = att;
%             	obj.R = vSO3.expmap(att);
%             	obj.q = Attitude.dcm2quat(obj.R);
            else
            	error('3rd input should have the size of [3,3] or [4,1] or [3,1]')
            end
            obj.Position = p_;
            obj.Velocity = v_;
            obj.biasAccel = ba_;
            obj.biasGyro = bg_;
            obj.gVec = g_;
        end
        %-----------------------------------------------------------
        
        function y = gravityWGS84(lat, h)
        
        end
        %-----------------------------------------------------------
    end
end
