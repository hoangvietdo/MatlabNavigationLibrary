%% License: intelligent Navigation and Control System Laboratory (iNCLS) - Sejong University
%  Author : Viet
%  e-Mail : hoangvietdo@sju.ac.kr
%  Method Index : skewMatrix, dcmExpMap, dcmLogMap, quatExpMap, quatLogMap

%% TODO

%%
classdef vSO3
    
    % This class contains so called Lie Algebra of SO(3) Group ~ 3D Special
    % Orthogonal Group
    
    properties
    end
    
    methods(Static)
        %-----------------------------------------------------------
        function skew = skewMatrix(vector)
            if(size(vector,1) ~= 3 || size(vector,2) ~= 1)
                error('Input vector must be 3x1');
            end
            
            skew = [        0         ,   -vector(3)   ,    vector(2)  ;...
                vector(3)    ,          0          ,   -vector(1) ;...
                -vector(2)  ,     vector(1)   ,         0         ];
        end
        %-----------------------------------------------------------
        
        function y = dcmExpMap()
            
        end
        %-----------------------------------------------------------
        
        function y = dcmLogMap()
            
        end
        %-----------------------------------------------------------
        
        function y = quatExpMap()
            
        end
        %-----------------------------------------------------------
        
        function y = quatLogMap()
            
        end
        %-----------------------------------------------------------
        
    end
end