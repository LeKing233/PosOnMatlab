classdef ImuHandler <handle
    %IMUHANDLER Imu事务管理类
    %   封装Imu数据导入导出、计算结果绘图
    
    properties(Access = public)
        mFilePath;%文件地址
        mRawData;%原始数据
        mWSeq;%角速度数据
        mFSeq;%加速度数据
        mMSeq;%磁力计数据
        mTSeq;%时间戳数据
        phi0;%初始角度
        mSeqLength;%数据条目数

    end
    
    methods(Access = public)
        % @brief ImuHandler构造函数 
        % @param filePath .csv数据文件路径
        % @retval obj
        function obj = ImuHandler(filePath)
          obj.mFilePath = filePath;
          obj.extractData();
        end
        
        % @brief 提取数据 
        % @param None
        % @retval None
        function extractData(obj)
            obj.mRawData = readtable(obj.mFilePath);            
            obj.mWSeq = [obj.mRawData.AngularVelX';
                        obj.mRawData.AngularVelY';
                        obj.mRawData.AngularVelZ'];
            obj.mFSeq = [obj.mRawData.AccX';
                        obj.mRawData.AccY';
                        obj.mRawData.AccZ'];     
            obj.mMSeq = [obj.mRawData.MagX';
                        obj.mRawData.MagY';
                        obj.mRawData.MagZ'];
%             obj.phi0 = [obj.mRawData.AngleX(1);
%                     obj.mRawData.AngleY(1);
%                     obj.mRawData.AngleZ(1)];
            obj.mTSeq = obj.mRawData.Timestamp';  
            obj.checkFrameDrop();        
            obj.interpolateMissingFrames();
            obj.sensorCalibration();
            obj.init();
        end

        % @brief 传感器零偏校准 
        % @param None
        % @retval None
        function sensorCalibration(obj)
            winLen = int32(2000 /6);%校准选取的窗口时间长度（ms）
            size1 = size(obj.mWSeq);
            wSeqInWin = obj.mWSeq(:,1:winLen);
            sensorDeviationValue = mean(wSeqInWin');%偏差值
            obj.mWSeq = obj.mWSeq - sensorDeviationValue';
        end
        
        % @brief 初始化 
        % @param None
        % @retval None
        function init(obj)
            obj.mSeqLength = size(obj.mWSeq,2);
            obj.GaitDtrInit();
        end
        
    end
    %% 步态检测
    properties(Access = public)
        mGaitDtr;%步态相位检测器
    end    
      methods(Access = public)     

        % @brief 计算第index帧步态数据 
        % @param index 迭代器当前索引
        % @retval None
        function phase = getPhase(obj,index)            
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
                phase = Utils.Phase_Stance;%支撑相
            else
                phase = Utils.Phase_Swing;%摆动相
            end

            %将结果保存进序列
            obj.mGaitDtr.gaitPhaseSeq(index) = phase;
            obj.mGaitDtr.T_Seq(index) = T;
        end
    end


    methods(Access = private)
        
        % @brief 步态检测器初始化
        % @param None
        % @retval None
        function GaitDtrInit(obj)
            obj.mGaitDtr.threshold = 7.8;%阈值
            obj.mGaitDtr.noise_w = 20 * ones(3,1);%陀螺仪测量方差向量
            obj.mGaitDtr.noise_f = 10 * ones(3,1);%加速度计测量方差向量
            obj.mGaitDtr.gravity = 9.8;%加速度数值
            obj.mGaitDtr.windowLength = 10;%窗口长度
            %初始化结果序
            obj.mGaitDtr.gaitPhaseSeq = zeros(obj.mSeqLength,1);%步态检测结果序列-初始化分配内存
            obj.mGaitDtr.T_seq = zeros(obj.mSeqLength,1);%统计量序列初始化分配内存
        end

        % @brief 计算步态统计量
        % @param wWindowSeq 角速度数据窗口序列   fWindowSeq 加速度数据窗口序列
        % @retval None
        function T = getT(obj,wWindowSeq,fWindowSeq)
            T = 0;    
            f_mean = mean(fWindowSeq);%加速度均值
            windowLength = length(fWindowSeq);%窗口长度
            
            for i = 1:windowLength
                T = T + 1/windowLength * ...
                        ( ...
                            1/norm(obj.mGaitDtr.noise_w)^2 * norm(wWindowSeq(:,i))^2 +...
                            1/norm(obj.mGaitDtr.noise_f)^2 * norm(fWindowSeq(:,i) - obj.mGaitDtr.gravity*f_mean/norm(f_mean))^2 ...
                        );
            end
        end 
        
        % @brief 获取窗口序列——提取第index帧前的窗口长度数据
        % @param dataSeq 三维数据向量     index 迭代器当前索引
        % @retval None
        function WinData = getWinData(obj,dataSeq,index)
            WinData = dataSeq(:,index-(obj.mGaitDtr.windowLength-1):index);
        end
        
        % @brief 检测数据序列丢帧
        % @param None
        % @retval None
        function checkFrameDrop(obj)
            % 计算时间戳之间的差值
            time_diff = diff(obj.mTSeq);
            % 找出大于6ms的时间差
            frame_drops = find(time_diff > 6);
            if isempty(frame_drops)
                disp('没有丢帧');
            else
                disp('丢帧的位置和对应的时间戳值如下：');
                for i = 1:length(frame_drops)
                    index = frame_drops(i);
                    timestamp = obj.mTSeq(index);
                    frame_interval = time_diff(index);
                    fprintf('位置: %d, 时间戳: %d, 帧间隔: %d ms\n', index, timestamp, frame_interval);
                end
            end
        end
        
        
        % @brief 补齐丢失数据帧
        % @param None
        % @retval None
        function interpolateMissingFrames(obj)
            index = 1;
            while index < length(obj.mTSeq) - 1
                time_diff = obj.mTSeq(index+1) - obj.mTSeq(index);
                if time_diff > 6
                    % 计算丢失的帧数
                    num_missing_frames = round(time_diff / 6);
                    % 计算丢失帧的时间戳
                    missing_timestamps = obj.mTSeq(index) + (1:num_missing_frames) * 6;
                    % 对每个数据序列进行线性插值
                    wSeq1 = [obj.mWSeq(1, 1:index), interp1(obj.mTSeq(index:index+1), obj.mWSeq(1, index:index+1), missing_timestamps), obj.mWSeq(1, index+2:end)];
                    wSeq2 = [obj.mWSeq(2, 1:index), interp1(obj.mTSeq(index:index+1), obj.mWSeq(2, index:index+1), missing_timestamps), obj.mWSeq(2, index+2:end)];
                    wSeq3 = [obj.mWSeq(3, 1:index), interp1(obj.mTSeq(index:index+1), obj.mWSeq(3, index:index+1), missing_timestamps), obj.mWSeq(3, index+2:end)];
                    fSeq1 = [obj.mFSeq(1, 1:index), interp1(obj.mTSeq(index:index+1), obj.mFSeq(1, index:index+1), missing_timestamps), obj.mFSeq(1, index+2:end)];
                    fSeq2 = [obj.mFSeq(2, 1:index), interp1(obj.mTSeq(index:index+1), obj.mFSeq(2, index:index+1), missing_timestamps), obj.mFSeq(2, index+2:end)];
                    fSeq3 = [obj.mFSeq(3, 1:index), interp1(obj.mTSeq(index:index+1), obj.mFSeq(3, index:index+1), missing_timestamps), obj.mFSeq(3, index+2:end)];
                    mSeq1 = [obj.mMSeq(1, 1:index), interp1(obj.mTSeq(index:index+1), obj.mMSeq(1, index:index+1), missing_timestamps), obj.mMSeq(1, index+2:end)];
                    mSeq2 = [obj.mMSeq(2, 1:index), interp1(obj.mTSeq(index:index+1), obj.mMSeq(2, index:index+1), missing_timestamps), obj.mMSeq(2, index+2:end)];
                    mSeq3 = [obj.mMSeq(3, 1:index), interp1(obj.mTSeq(index:index+1), obj.mMSeq(3, index:index+1), missing_timestamps), obj.mMSeq(3, index+2:end)];
                    obj.mWSeq = [wSeq1; wSeq2; wSeq3];
                    obj.mFSeq = [fSeq1; fSeq2; fSeq3];
                    obj.mMSeq = [mSeq1; mSeq2; mSeq3];
                    % 更新时间戳序列
                    obj.mTSeq = [obj.mTSeq(1:index), missing_timestamps, obj.mTSeq(index+2:end)];
                end
                index = index + 1;
            end
        end




    end     

end
