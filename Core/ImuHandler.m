classdef ImuHandler <handle
    %IMUHANDLER Imu事务管理类
    %   封装Imu数据导入导出、计算结果绘图
    
    properties(Access = public)
        mFilePath;%文件地址
        mRawData;%原始数据
        mWSeq;
        mFSeq;
        mSeqLength;%数据条目数
        

    
    end
    
    methods(Access = public)
        function obj = ImuHandler(filePath)
          obj.mFilePath = filePath;
          obj.extractData();
          obj.init();
          obj.calculateRequiredData();
        end
        
        %提取数据
        function extractData(obj)
            obj.mRawData = readtable(obj.mFilePath);
            obj.mWSeq = [obj.mRawData.AngularVelX';
                         obj.mRawData.AngularVelY';
                         obj.mRawData.AngularVelZ'];
            obj.mFSeq = [obj.mRawData.AccX';
                         obj.mRawData.AccY';
                         obj.mRawData.AccZ'];

        end
        %初始化
        function init(obj)
            obj.mSeqLength = size(obj.mRawData,1);
            obj.GaitDtrInit();            
        end

        %预计算需要的结果
        function calculateRequiredData(obj)
            obj.calculateGaitPhaseSeq();
        end

        
        



    end
    %% 步态检测
    properties(Access = public)
        mGaitDtr;%步态相位检测器
    end    

    %私有方法
    methods(Access = private)
         %步态检测器初始化
        function GaitDtrInit(obj)
            obj.mGaitDtr.threshold = 7.8;%阈值
            obj.mGaitDtr.noise_w = 20 * ones(3,1);%陀螺仪测量方差向量
            obj.mGaitDtr.noise_f = 10 * ones(3,1);%加速度计测量方差向量
            obj.mGaitDtr.gravity = 9.8;%加速度数值
            obj.mGaitDtr.windowLength = 5;%窗口长度
            %初始化结果序
            obj.mGaitDtr.gaitPhaseSeq = zeros(obj.mSeqLength,1);%步态检测结果序列-初始化分配内存
            obj.mGaitDtr.T_seq = zeros(obj.mSeqLength,1);%统计量序列初始化分配内存
        end


        function T = getT(obj, wWindowSeq, varargin)
            T = 0;          
            windowLength = length(wWindowSeq);
        
            if isempty(varargin)
                % 只有 wWindowSeq 参数的情况，计算统计量——单纯基于角速度w的检测条件                
                for i = 1:windowLength
                    T = T + 1/windowLength *(1/norm(obj.mGaitDtr.noise_w)^2 * norm(wWindowSeq(:,i))^2);
                end
            else
                % 存在 fWindowSeq 参数的情况 ，基于加速度a和角速度w的混合检测条件
                fWindowSeq = varargin{1};
                f_mean = mean(fWindowSeq); % 加速度均值
                for i = 1:windowLength
                    T = T + 1/windowLength * ...
                            ( ...
                                1/norm(obj.mGaitDtr.noise_w)^2 * norm(wWindowSeq(:,i))^2 +...
                                1/norm(obj.mGaitDtr.noise_f)^2 * norm(fWindowSeq(:,i) - obj.mGaitDtr.gravity*f_mean/norm(f_mean))^2 ...
                            );
                end
            end
        
        end

       
        
        %获取窗口序列——提取第index帧前的窗口长度数据
        %dataSeq-三维数据向量
        function WinData = getWinData(obj,dataSeq,index)
            WinData = dataSeq(:,index-(obj.mGaitDtr.windowLength-1):index);
        end

         %计算第index帧数据
        function phase = calculatePhase(obj,index)            
            if index < obj.mGaitDtr.windowLength
                phase = Utils.Phase_Unknown;
                obj.mGaitDtr.gaitPhaseSeq(index) = phase;
                return;
            end
            wWinData = obj.getWinData(obj.mWSeq,index);
            fWinData = obj.getWinData(obj.mFSeq,index);
            
            T = obj.getT(wWinData,fWinData);
            
            % 根据统计量获取步相
            if T <  obj.mGaitDtr.threshold
                phase = Utils.Phase_Static;%静止
            else
                phase = Utils.Phase_Active;%活动
            end

            % 将结果保存进序列
            obj.mGaitDtr.gaitPhaseSeq(index) = phase;
            obj.mGaitDtr.T_Seq(index) = T;
        end
        
        %计算GaitPhase序列
        function calculateGaitPhaseSeq(obj)
            for i = 1:obj.mSeqLength                
                obj.calculatePhase(i);
            end
        end
     end
    
    % 公有方法
    methods(Access = public)     
        function phase = getPhase(obj,index)
            phase = obj.mGaitDtr.gaitPhaseSeq(index);
        end
    end


   

    %% 绘图函数
        


end


