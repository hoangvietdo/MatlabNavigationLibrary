%% License: intelligent Navigation and Control System Laboratory (iNCLS) - Sejong University
%  Author : Viet
%  e-Mail : hoangvietdo@sju.ac.kr
%  Method Index : homoMatrix, homoInvMatrix, homo2homoInv

%% TODO

%%
classdef vSE3
    
    % Special Euclidean 3D Group SE(3)
    
    properties
    end
    
    methods(Static)
        %-----------------------------------------------------------
        function homo = homoMatrix(R, t)
            validateattributes(R,{'double'},{'size', [3,3]})
            validateattributes(t,{'double'},{'size', [3,1]})
            
            homo = eye(4,4);
            homo(1:3, 1:3) = R;
            homo(1:3, 4) = t;
        end
        %-----------------------------------------------------------
        
        function homo = homoInvMatrix(R, t)
            validateattributes(R,{'double'},{'size', [3,3]})
            validateattributes(t,{'double'},{'size', [3,1]})
            
            homo = eye(4,4);
            homo(1:3, 1:3) = R';
            homo(1:3, 4) = -R' * t;
        end
        %-----------------------------------------------------------
        
        function homoInv = homo2homoInv(homo)
            validateattributes(homo,{'double'},{'size', [4,4]})
            
            homoInv = eye(4,4);
            homoInv(1:3, 1:3) = homo(1:3, 1:3)';
            homoInv(1:3, 4) = -homo(1:3, 1:3)' * homo(1:3, 4);
        end
        %-----------------------------------------------------------
        
    end
end