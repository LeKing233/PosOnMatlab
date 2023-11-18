classdef GaitPhaseDetector <handle
    properties        
        %计算量
        T;%计算统计量序列
        GaitPhase;%步态相位

        %常量
        threshold;%阈值
        noise_w;%陀螺仪测量方差向量
        noise_f;%加速度计测量方差向量
        gravity;%加速度数值
        windowLength;%窗口长度

        PhaseStance = 0;%支撑相
        PhaseSwing = 1;%摆动相
        PhaseUnknown = -1;%未知情况

    end

    methods
        %构造函数
        function obj = GaitPhaseDetector()       
            obj.threshold = 7.8;%阈值
            obj.noise_w = 20 * ones(3,1);%陀螺仪测量方差向量
            obj.noise_f = 10 * ones(3,1);%加速度计测量方差向量
            obj.gravity = 9.8;%加速度数值
            obj.windowLength = 10;%窗口长度

            obj.init();
        end
        %检测器初始化
        function init(obj)
            obj.GaitPhase = obj.PhaseUnknown;
        end
        
        %根据统计量计算步态相位结果
        function phase = getPhase(obj,wSeq,fSeq)            
            if length(wSeq) <= obj.windowLength
                obj.GaitPhase = obj.PhaseUnknown;
                phase = obj.GaitPhase;
                return;
            end
            
            obj.calculateTSeq(obj.extractWinData(wSeq),obj.extractWinData(fSeq));
            
            % 根据统计量获取步相
            if obj.T <  obj.threshold
                obj.GaitPhase = obj.PhaseStance;%支撑相
            else
                obj.GaitPhase = obj.PhaseSwing;%摆动相
            end

            phase = obj.GaitPhase; 

        end
        
         %计算基于滑动窗口的统计量
        function T = calculateTSeq(obj,wWindowSeq,fWindowSeq)
            T = 0;    
            f_mean = mean(fWindowSeq);%加速度均值
            windowLength = length(fWindowSeq);%窗口长度
            
            for i = 1:windowLength
                T = T + 1/windowLength * ...
                        ( ...
                            1/norm(obj.noise_w)^2 * norm(wWindowSeq(:,i))^2 +...
                            1/norm(obj.noise_f)^2 * norm(fWindowSeq(:,i) - obj.gravity*f_mean/norm(f_mean))^2 ...
                        );
            end
            obj.T = T;
        end
        
        %获取窗口序列
        function WinData = extractWinData(obj,dataSeq)
            seqLength = length(dataSeq);
            WinData = dataSeq(:,(seqLength-obj.windowLength):seqLength);
        end
        
    end
end