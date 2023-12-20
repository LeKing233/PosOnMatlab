classdef Fitter
    %拟合器，用于拟合步行速度和足底压力之间的关系公式
    properties

    end

    methods
        function obj = Fitter()
            
        end
    end




    methods(Static)
        % @brief 拟合出速度和足底压力之间的相关关系，通过多次获得的pHandler数据，和得到的速度数据，
        % 速度（km/h)
        % 压力峰值
        % @param pHandlerArray 足底压力数据数组
        function fit_Speeds_peaks(pHandlerArray)
            pHandlerArray = Plotter.toArray(pHandlerArray);
            %预分配速度和压力的序列
            speeds = zeros(length(pHandlerArray),1);
            peaks = zeros(length(pHandlerArray),1);
            for i = 1:length(pHandlerArray)
                pHandler = pHandlerArray(i);
                SumSeqInArea = pHandler.mSumSeqInArea;
                speeds(i) = pHandler.mWalkSpeed;
                [Maxpeaks,locs] = findpeaks(SumSeqInArea.T_peaks)
                peaks(i) = mean(Maxpeaks);
            end
            %拟合
            cftool(peaks,speeds)
        end
        %------------------【T区域】--------------------       
        %-----------[一次多项式]--------------
        % 线性模型 Poly1:
        %      f(x) = p1*x + p2
        % 系数(置信边界为 95%):
        %        p1 =     0.07218  (0.05775, 0.08661)
        %        p2 =      -1.491  (-3.004, 0.02248)
        % 
        % 拟合优度:
        %   SSE: 61.18
        %   R 方: 0.7766
        %   调整 R 方: 0.7692
    end
end