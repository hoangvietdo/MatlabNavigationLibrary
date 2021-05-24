%% License: intelligent Navigation and Control System Laboratory (iNCLS) - Sejong University
%  Author : Viet
%  e-Mail : hoangvietdo@sju.ac.kr
%  Method Index : dcm2euler , dcm2quat , dcmNormalize , euler2dcm , euler2quat
%             quat2dcm , quatConjugate , quatMultiply , quatLeftMultiply
%             quatRightMultiply , quatIntegrateRK4 , omegaMat, rvec2quat , quatCorrectionIKF

%% TODO

%%
classdef Attitude
    
    % Attitude class includes all the transformations and math operations that related to the
    % attitude which are euler, direction cosine matrix (dcm), quaternion,
    
    % Math Equations and Notations used in this class are mainly from:
    % Reference:  1. Titterton, D. and J. Weston, STRAPDOWN
    %                        INERTIAL NAVIGATION TECHNOLOGY, Peter
    %                        Peregrinus Ltd. on behalf of the Institution
    %                        of Electrical Engineers, London, 1997.
    %                   2.  M. D. Shuster. Survey of attitude representations. Journal of the
    %                        Astronautical Sciences, 41(4):439–517, 1993.
    %                   3. Savage, Paul G. Strapdown analytics. Vol. 2. Maple Plain, 
    %                       MN: Strapdown Associates, 2000.
    
    properties
    end
    
    methods(Static)
        
        %-----------------------------------------------------------
        function euler = dcm2euler(dcm)
            
            %
            % Explanation: This function converts from Direction Cosine Matrix (DCM) to
            %                     Euler Angle 321 sequence
            %                     if you have dcm(R_{2}^{1}), this function calculates euler angle
			%                     that rotates frame {1} by (z,psi), (y,theta), (x,phi) to frame {2}.
            %
            % Input:  A [3x3] DCM Cbn that transforms Pose in the Body Frame (b) to the navigation frame (n)
            %
            %           Cbn = 11 12 13
            %                 21 22 23
            %                 31 32 33
            %
            % Output: 3 Euler angles = Roll (phi), Pitch (theta), Yaw (psi)
            %
            %         euler_angle(1) = roll (rad)
            %         euler_angle(2) = pitch (rad)
            %         euler_angle(3) = yaw (rad)
            %
            % Note:
            %         If the pitch angle (theta) is vanishingly close to +/- pi/2,
            %         the elements of euler_angle will be filled with NaN (indeterminate).
            %         -pi/2 <= pitch <= pi/2
            %         if dcm(3,1) <= -0.999 then
            %         (heading - roll) will be stored in heading and NaN in roll
            %         if dcm(3,1) >= 0.999
            %         (heading + roll) will be stored in heading and NaN in roll
            %         Ref: Starpdown Analytics by Savage.
            
            if (nargin < 1)
                error('insufficient number of input arguments!')
            end
            
            siz = size(dcm);
            
            if (siz(1) > 3 || siz(1) < 3 || siz(2) > 3 || siz(2) < 3)
                error('Invalid Matrix Dimension Input!')
            end

            pitch = atan(-dcm(3,1)/sqrt(dcm(3,2)^2 + dcm(3,3)^2));

            if dcm(3,1) <= -0.999
                roll = NaN;
                heading = atan2((dcm(2,3)-dcm(1,2)),(dcm(1,3)+dcm(2,2)));
            elseif dcm(3,1) >= 0.999
                roll = NaN;
                heading = pi + atan2((dcm(2,3)+dcm(1,2)),(dcm(1,3)-dcm(2,2)));
            else
                roll = atan2(dcm(3,2), dcm(3,3));
                heading = atan2(dcm(2,1), dcm(1,1));
            end

            euler = [roll; pitch; heading];
        end
        %-----------------------------------------------------------
        
        function quaternion = dcm2quat(dcm)
            
            % Ref 1 - Page: 45
            % Ref 2 - Equation: 157 - 168
            
            % Explanation: This function converts from Direction Cosine Matrix (dcm) to
            %              quaternion
            %
            % Input:  A [3x3] Direction Cosine Matrix (dcm)
            %
            %           dcm = 11 12 13
            %                 21 22 23
            %                 31 32 33
            %
            % Output: Quaternion [4x1] Matrix = [a b c d]' where a is a scalar component
            
            if (nargin < 1)
                error('insufficient number of input arguments!')
            end
            
            siz = size(dcm);
            
            if (siz(1) > 3 || siz(1) < 3 || siz(2) > 3 || siz(2) < 3)
                error('Invalid Matrix Dimension Input!')
            end
            
            % Ref 1
            %     a = 0.5*sqrt( 1 + dcm(1,1) + dcm(2,2) + dcm(3,3));
            %     b = (1/(4*a)) * (dcm(3,2) - dcm(2,3));
            %     c = (1/(4*a)) * (dcm(1,3) - dcm(3,1));
            %     d = (1/(4*a)) * (dcm(2,1) - dcm(1,2));
            %
            %     quaternion = [a b c d]';
            
            % Ref 2
            d = diag(dcm);
            q = zeros(4,1);
            
            q(1) = sqrt(1 / 4 * (1 + d(1) + d(2) +d(3)));
            q(2) = sqrt(1 / 4 * (1 + d(1) - d(2) -d(3)));
            q(3) = sqrt(1 / 4 * (1 - d(1) + d(2) -d(3)));
            q(4) = sqrt(1 / 4 * (1 - d(1) - d(2) +d(3)));
            
            % Building quaternion elements from subdiagonals
            max_idx = 1;
            for i=2:4
                if q(i) > q(max_idx)
                    max_idx = i;
                end
            end
            
            quaternion = zeros(4,1);
            
            switch max_idx
                case 1
                    quaternion(1) = q(1);
                    quaternion(2) = (dcm(3,2) - dcm(2,3)) / 4 / quaternion(1);
                    quaternion(3) = (dcm(1,3) - dcm(3,1)) / 4 / quaternion(1);
                    quaternion(4) = (dcm(2,1) - dcm(1,2)) / 4 / quaternion(1);
                case 2
                    quaternion(2) = q(2);
                    quaternion(1) = (dcm(3,2) - dcm(2,3)) / 4 / quaternion(2);
                    quaternion(3) = (dcm(2,1) + dcm(1,2)) / 4 / quaternion(2);
                    quaternion(4) = (dcm(1,3) + dcm(3,1)) / 4 / quaternion(2);
                case 3
                    quaternion(3) = q(3);
                    quaternion(1) = (dcm(1,3) - dcm(3,1)) / 4 / quaternion(3);
                    quaternion(2) = (dcm(2,1) + dcm(1,2)) / 4 / quaternion(3);
                    quaternion(4) = (dcm(3,2) + dcm(2,3)) / 4 / quaternion(3);
                case 4
                    quaternion(4) = q(4);
                    quaternion(1) = (dcm(2,1) - dcm(1,2)) / 4 / quaternion(4);
                    quaternion(2) = (dcm(1,3) + dcm(3,1)) / 4 / quaternion(4);
                    quaternion(3) = (dcm(3,2) + dcm(2,3)) / 4 / quaternion(4);
            end
            
            % Keep q0(scalar) be positive.
            if quaternion(1) < 0
                quaternion = -quaternion;
            end
        end
        %-----------------------------------------------------------
        
        function dcmNorm = dcmNormalize(dcm)
            
            % Normalize direction cosine matrix dcm
            % to make sure it is an orthogonal matrix det(dcm) = 1
            
            [U, S, V] = svd(dcm);
            dcmNorm = U * eye(size(S,1)) * V';
            
        end
        %-----------------------------------------------------------
        
        function dcm = euler2dcm(euler_angle)
            
            % Ref 1 - Page: 41
            %
            % Explanation: This function converts from Euler Angle to
            %              Direction Cosine Matrix (DCM)
            %
            % Input:  A [3x1] Euler Angle Matrix = Roll (phi), Pitch (theta), Yaw (psi)
            %
            %         euler_angle = a
            %                       b
            %                       c
            %
            % Output: A [3x3] Direction Cosine Matrix
            
            if (nargin < 1)
                error('insufficient number of input arguments!')
            end
            
            siz = size(euler_angle);
            
            if (siz(1) > 3 || siz(1) < 3 || siz(2) > 1 || siz(2) < 1)
                error('Invalid Matrix Dimension Input!')
            end
            
            phi = euler_angle(1);
            theta = euler_angle(2);
            psi = euler_angle(3);
            
            cs = cos(psi)   ; ss = sin(psi);
            ct = cos(theta) ; st = sin(theta);
            cp = cos(phi)   ; sp = sin(phi);
            
            C1 = [cs  ss   0; ...
                -ss   cs   0; ...
                0    0    1];
            
            C2 = [ct   0    -st; ...
                0    1    0; ...
                st  0    ct];
            
            C3 = [1   0    0;   ...
                0   cp   sp; ...
                0   -sp  cp];
            
            dcm = C1' * C2' * C3';
        end
        %-----------------------------------------------------------
        
        function quaternion = euler2quat(euler_angle)
            
            % Ref 1 - Page: 46
            %
            % Explanation: This function converts from Euler Angle to Quaternion
            %
            % Input:  A [3x1] Euler Angle Matrix = Roll (phi), Pitch (theta), Yaw (psi)
            %
            %         euler_angle = a
            %                       b
            %                       c
            %
            % Output: Quaternion [4x1] Matrix
            
            if (nargin < 1)
                error('insufficient number of input arguments!')
            end
            
            siz = size(euler_angle);
            
            if (siz(1) > 3 || siz(1) < 3 || siz(2) > 1 || siz(2) < 1)
                error('Invalid Matrix Dimension Input!')
            end
            
            phi = euler_angle(1);
            theta = euler_angle(2);
            psi = euler_angle(3);
            
            cs = cos(psi/2)   ; ss = sin(psi/2);
            ct = cos(theta/2) ; st = sin(theta/2);
            cp = cos(phi/2)   ; sp = sin(phi/2);
            
            a = cp * ct * cs + sp * st * ss;
            b = sp * ct * cs - cp * st * ss;
            c = cp * st * cs + sp * ct * ss;
            d = cp * ct * ss - sp * st * cs;
            
            quaternion = [a b c d]';
            
        end
        %-----------------------------------------------------------
        
        function dcm = quat2dcm(quaternion)
            
            % Ref 1 - Page: 44
            %
            % Explanation: This function converts from Direction Cosine Matrix (dcm) to quaternion
            %
            % Input: A [4x1] Quaternion Matrix = [a b c d]'
            %
            % Output:  A [3x3] Direction Cosine Matrix (dcm)
            %
            %           dcm = 11 12 13
            %                 21 22 23
            %                 31 32 33
            
            if (nargin < 1)
                error('insufficient number of input arguments!')
            end
            
            siz = size(quaternion);
            
            if (siz(1) > 4 || siz(1) < 4 || siz(2) > 1 || siz(2) < 1)
                error('Invalid Matrix Dimension Input!')
            end
            
            a = quaternion(1);
            b = quaternion(2);
            c = quaternion(3);
            d = quaternion(4);
            
            a1 = a * a + b * b - c * c - d * d;
            a2 = 2 * (b * c - a * d);
            a3 = 2 * (b * d + a * c);
            
            b1 = 2 * (b * c + a * d);
            b2 = a * a - b * b + c * c - d * d;
            b3 = 2 * (c * d - a * b);
            
            c1 = 2 * (b * d - a * c);
            c2 = 2 * (c * d + a * b);
            c3 = a * a - b * b - c * c + d * d;
            
            dcm = [ a1 a2 a3; ...
                b1 b2 b3; ...
                c1 c2 c3 ];
            
        end
        %-----------------------------------------------------------
        
        function quatConj = quatConjugate(quat)
            
            % Calculate the Conjugate of the given quaternion
            quatConj = [quat(1); -quat(2:4)];
            
        end
        %-----------------------------------------------------------
        
        function quatNorm = quatNormalize(quat)
            
            % Normalize quaternion to make sure norm 2 of quaternion |q|_2 = 1
            quatNorm = quat / norm(quat);
            
        end
        %-----------------------------------------------------------
        
        function y = quatMultiply(q, p)
            
            % Ref 1 - Page: 43
            %
            % Input:  2 Quaternion vector [4x1];
            
            % Output: Quaternion [4x1] Matrix
            
            if (nargin < 2)
                error('insufficient number of input arguments!')
            end
            
            siz_q = size(q);
            
            if (siz_q(1) > 4 || siz_q(1) < 4 || siz_q(2) > 1 || siz_q(2) < 1)
                error('Invalid Matrix Dimension Input! (q)')
            end
            
            siz_p = size(p);
            
            if (siz_p(1) > 4 || siz_p(1) < 4 || siz_p(2) > 1 || siz_p(2) < 1)
                error('Invalid Matrix Dimension Input! (p) ')
            end
            
            a = q(1) ;
            b = q(2) ;
            c = q(3) ;
            d = q(4) ;
            
            temp = [ a , -b  , -c  ,  -d ;...
                b ,  a  , -d  ,   c ;...
                c ,  d  ,  a  ,  -b ;...
                d , -c  ,  b  ,   a ];
            
            y = temp*p;
            
            if y(1,1)<0
                y = -y;
            end
            
            y = Attitude.quatNormalize(y); % keep the quaternion fresh ! ^_^
        end
        %-----------------------------------------------------------
        
        function y = quatLeftMultiply(quaternion)
            
            scalar = quaternion(1);
            q_vec = [quaternion(2); quaternion(3); quaternion(4)];
            
            y = [scalar, -q_vec'; ...
                q_vec, scalar * eye(3) + skewMatrix(q_vec)];
            
            y = Attitude.quatNormalize(y);
        end
        %-----------------------------------------------------------
        
        function y = quatRightMultiply(quaternion)
            
            scalar = quaternion(1);
            q_vec = [quaternion(2); quaternion(3); quaternion(4)];
            
            y = [scalar, -q_vec'; ...
                q_vec, scalar * eye(3) - skewMatrix(q_vec)];
            
            y = Attitude.quatNormalize(y);
        end
        %-----------------------------------------------------------
        
        function y = quatIntegrateRK4(ql, wl, wl1, dt)
            
            dq1 = [0; wl];
            dq2 = [0; (wl + wl1) / 2];
            dq3 = [0; wl1];
            
            Om1 = quatRightMultiply(dq1);
            Om2 = quatRightMultiply(dq2);
            Om3 = quatRightMultiply(dq3);
            
            k1 = 1/2 * Om1*ql;
            k2 = 1/2 * Om2 * (ql + dt / 2 * k1);
            k3 = 1/2 * Om2 * (ql + dt / 2 * k2);
            k4 = 1/2 * Om3 * (ql + dt * k3);
            
            y = ql +  dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
            y = Attitude.quatNormalize(y);
            
        end
        %-----------------------------------------------------------
        
        function bigOmega = omegaMat(omega, Opts)
            
            % Calculate the big omega matrix for INS mechanization
            % There are two options for this function which relate to
            % quaternion representation
            % 1. Hamilton
            % 2. JPL
            if(size(omega,1) ~= 3 || size(omega,2) ~= 1)
                error('Input vector must be 3x1');
            end
            
            switch(Opts)
                case 'Hamilton'
                    bigOmega = [ vSO3.skewMatrix(-omega),  omega;...
                        -omega',            0 ];
                case 'JPL'
                    bigOmega = [     0     ,   -omega';...
                        omega ,   vSO3.skewMatrix(-omega)];
                otherwise
                    error('Not a proper option')
            end
        end
        %-----------------------------------------------------------
        
        function quat = rvec2quat(rvec)
            
            % Calculate Magnitude
            mag = norm(rvec);
            
            % Ref 1 - Page 42
            if (mag == 0)
                quat = [1;0;0;0];
            else
                cs = cos(mag/2);
                ss = sin(mag/2);
                k   = rvec/mag;
                
                quat = [cs; ss*k];
            end
        end
        %-----------------------------------------------------------
        
        function quatCorrected = quatCorrectionIKF(quat, deltaAttitude)
            
            % Ref 1 - Page 407
            a = quat(1);
            b = quat(2);
            c = quat(3);
            d = quat(4);
            
%             fooMatrix = [ b c d;...
%                 -a d -c;...
%                 -d -a b;...
%                 c -b -a];
            fooMatrix = [ -b -c -d;...
                a d -c;...
                -d a b;...
                c -b a];
            
            quatCorrected = quat + 0.5 * fooMatrix * deltaAttitude;
            quatCorrected = Attitude.quatNormalize(quatCorrected);
        end
        %-----------------------------------------------------------
        
        function euler = quat2euler(quat)
            
            foo = Attitude.quat2dcm(quat);
            euler = Attitude.dcm2euler(foo);
            
        end
        %-----------------------------------------------------------
        
        
        %-----------------------------------------------------------
    end
end