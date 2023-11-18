classdef UtilContainer <handle
    properties
        
    end

    methods
        function obj = UtilContainer()
            
        end
        % 计算三维向量的反对称矩阵
        function crossMatrix = getCrossMatrix(obj,v)
            crossMatrix = [0, -v(3), v(2);
                           v(3), 0, -v(1);
                           -v(2), v(1), 0];    
        end
        
    end
end