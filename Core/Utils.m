classdef Utils <handle
    

    %静态常量
    properties(Constant)
        

        %IMU用
        Phase_Static = 0.3;%静止
        Phase_Active = 0.7;%活动

        %plantar用
        Phase_HeelStrike = 0.6;%脚跟着地
        Phase_ToeOff = 0.4;%脚尖离地

        Phase_Landing = 0.2;%脚着地
        Phase_Floating = 0.8;%脚离地浮空

        Phase_Unknown = 0.5;%未知情况



        %最终的
        Phase_Stance = 0;%静止
        Phase_Swing = 1;%运动

        
    end

    methods
        function obj = Utils()
            
        end
        % 计算三维向量的反对称矩阵
        function crossMatrix = getCrossMatrix(obj,v)
            crossMatrix = [0, -v(3), v(2);
                           v(3), 0, -v(1);
                           -v(2), v(1), 0];    
        end
        
    end
end