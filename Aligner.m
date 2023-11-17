classdef Aligner
    %完成初始对准，包括粗对准和精对准
    properties
%         gravity = 9.8;%重力加速度数值
    end

    methods
        
        function obj = Aligner()
            
        end

        %粗对准
        function roughAlignment(obj,f,m)%f 加速度，m 磁力计数值
            gamma = atan(f(2)/f(3)) *180/pi
            theta = -asin(f(1)/norm(f))*180/pi
            g = norm(f)
        end
        
        %精确对准
        function accurateAlignment(obj)
            
        end

    end
end